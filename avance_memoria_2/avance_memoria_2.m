% MPC and MHE implementation dor a differential drive robot
% EKF comparison
% Simulates a significant state error in the first third of the trajectory
% ************************************************************************

clearvars
close all
clc

addpath(fullfile(pwd, 'casadi-3.6.5-windows64-matlab2018b'))
import casadi.*

% *************************************************************************
% General Parameters:
% *************************************************************************
Ts = 0.100;  % Sampling time
gps_noise_factor = 0.1 * 5;  % GPS noise factor standard deviation
gps_noise_factor_variance = gps_noise_factor^2;  % GPS noise factor variance

gyro_noise_factor = 2 * pi * 0.005;  % Gyro noise factor standard deviation
gyro_noise_factor_variance = gyro_noise_factor^2;  % Gyro noise factor variance

% *************************************************************************
% Vehicle's Geometrical Properties:
% *************************************************************************
w = 0.555;  % Robot width (between wheels)
l = 0.544;  % Robot length

wg = 0.6;  % GPS width
lg = 0.6;  % GPS length

% *************************************************************************
% System's Properties:
% *************************************************************************
nq = 3;  % Number of states (x, y, theta)
nu = 2;  % Number of inputs (vr, vl)
nr = 3;  % Number of reference states

% *************************************************************************
% Model Predictive Control (MPC) Parameters:
% *************************************************************************
Nc = 50;  % Control horizon length

U_lb = [-4; -4];  % Lower bound for the control inputs
U_ub = [4; 4];    % Upper bound for the control inputs

dU_lb = U_lb * 0.1;  % Lower bound for the acceleration
dU_ub = U_ub * 0.1;  % Upper bound for the acceleration

% Create Casadi optimization object
optiMPC = casadi.Opti();

% Define MPC variables
qMPC = optiMPC.variable(nq, Nc + 1);    % Predicted state trajectories
u = optiMPC.variable(nu, Nc);           % Control inputs
qref = optiMPC.parameter(nr);           % Reference state
qinit = optiMPC.parameter(nq);          % Initial state
u_last = optiMPC.parameter(nu);         % Last control input

% Cost matrices
Wmpc = diag([1, 1, 2] * 2);             % State cost weight matrix
Rmpc = diag([0.08, 0.08]);              % Control input cost weight matrix

% *************************************************************************
% MPC Objective Function
% *************************************************************************
Jmpc = 0;
for k = 1:Nc + 1
    % Error between predicted state and reference state
    error = qMPC(:, k) - qref;

    % Adjust the angle to be within the range [-pi, pi]
    error(3) = if_else(error(3) > pi, error(3) - 2 * pi, error(3));
    error(3) = if_else(error(3) < -pi, error(3) + 2 * pi, error(3));

    Jmpc = Jmpc + (error).' * Wmpc * (error);
    if k < Nc + 1
        optiMPC.subject_to(qMPC(:, k + 1) == F(qMPC(:, k), u(:, k), Ts, w, zeros(3, 1)));

        % Acceleration constraints
        if k ~= 1
            optiMPC.subject_to((dU_lb <= (u(:, k) - u(:, k - 1)) / Ts) <= dU_ub);
        else
            optiMPC.subject_to((dU_lb <= (u(:, k) - u_last) / Ts) <= dU_ub);
        end
        optiMPC.subject_to((U_lb <= u(:, k)) <= U_ub);
        Jmpc = Jmpc + u(:, k).' * Rmpc * u(:, k);
    end
end

% Forward movement constraint
optiMPC.subject_to(u(1, :) * 0.5 + u(2, :) * 0.5 >= 0);

% Seek minimum action to avoid oscillations
optiMPC.minimize(sum(sum(u.^2)));

% Set objective
optiMPC.minimize(Jmpc);

% Set initial state constraint
optiMPC.subject_to(qMPC(:, 1) == qinit);

% Solver options
p_optsMPC = struct('expand', true);
s_optsMPC = struct('sb', 'yes', 'print_level', 0, 'warm_start_init_point', 'yes');
optiMPC.solver('ipopt', p_optsMPC, s_optsMPC);

% *************************************************************************
% Moving Horizon Estimation (MHE) Parameters:
% *************************************************************************
Nh = 60;  % MHE horizon length
optiMHE = casadi.Opti();

qMHE = optiMHE.variable(nq, Nh + 1);        % State estimates
wMHE = optiMHE.variable(nq, Nh);            % Process noise
vMHE = optiMHE.variable(nq, Nh + 1);        % Measurement noise

slackGps1 = optiMHE.variable(nq, Nh + 1);   % Slack variables for GPS measurements
slackGps2 = optiMHE.variable(nq, Nh + 1);   % Slack variables for GPS measurements
slackGps3 = optiMHE.variable(nq, Nh + 1);   % Slack variables for GPS measurements
slackGps4 = optiMHE.variable(nq, Nh + 1);   % Slack variables for GPS measurements
slackmeanGPS = optiMHE.variable(nq, Nh + 1);% Slack variable for mean GPS measurements

y_meas = optiMHE.parameter(4 * 2 + 1, Nh + 1);  % Measurements

uPast = optiMHE.parameter(nu, Nh);          % Past control inputs applied to the system
X = optiMHE.variable(nq);                   % Arrival-cost
xBar = optiMHE.parameter(nq);               % Initial state for MHE

% Cost matrices for MHE
Rmhe = diag([gps_noise_factor, gps_noise_factor, gyro_noise_factor * 2]);  % Measurement cost
Qmhe = diag([1, 1, 100] * 0.01);            % Process noise cost
Gmhe = diag([1, 1, 10] * 0.00002);         % Slack cost (greater means faster but less accurate)(for 0.0001 factor the error is equal to EKF)
Pmhe = diag([gps_noise_factor, gps_noise_factor, 0.001]);  % Arrival cost

% *************************************************************************
% MHE Objective Function
% *************************************************************************
% Set initial state constraint
Jmhe = X.' * Pmhe * X;  % Arrival cost
optiMHE.subject_to(X == qMHE(:, 1) - xBar);

for k = 1:Nh
    Jmhe = Jmhe + vMHE(:, k).' * Rmhe * vMHE(:, k) + wMHE(:, k).' * Qmhe * wMHE(:, k) ...
        + slackGps1(:, k).' * Gmhe * slackGps1(:, k) ...
        + slackGps2(:, k).' * Gmhe * slackGps2(:, k) ...
        + slackGps3(:, k).' * Gmhe * slackGps3(:, k) ...
        + slackGps4(:, k).' * Gmhe * slackGps4(:, k);

    optiMHE.subject_to(qMHE(:, k + 1) == F(qMHE(:, k), uPast(:, k), Ts, w, wMHE(:, k)));

    % Rotation matrix
    R = [cos(y_meas(4 * 2 + 1, k)), -sin(y_meas(4 * 2 + 1, k)), 0;
         sin(y_meas(4 * 2 + 1, k)),  cos(y_meas(4 * 2 + 1, k)), 0;
         0,                          0,                          0];

    % GPS measurements
    optiMHE.subject_to(qMHE(:, k) + vMHE(:, k) == [y_meas(1:2, k); y_meas(4 * 2 + 1, k)] ...
        - R * 0.5 * [lg; -wg; 0] + slackGps1(:, k));
    optiMHE.subject_to(qMHE(:, k) + vMHE(:, k) == [y_meas(3:4, k); y_meas(4 * 2 + 1, k)] ...
        - R * 0.5 * [lg; wg; 0] + slackGps2(:, k));
    optiMHE.subject_to(qMHE(:, k) + vMHE(:, k) == [y_meas(5:6, k); y_meas(4 * 2 + 1, k)] ...
        - R * 0.5 * [-lg; wg; 0] + slackGps3(:, k));
    optiMHE.subject_to(qMHE(:, k) + vMHE(:, k) == [y_meas(7:8, k); y_meas(4 * 2 + 1, k)] ...
        - R * 0.5 * [-lg; -wg; 0] + slackGps4(:, k));
    % GPS mean
    optiMHE.subject_to(qMHE(:, k) + vMHE(:, k) == [mean([y_meas(1:2, k), y_meas(3:4, k), y_meas(5:6, k), y_meas(7:8, k)], 2); y_meas(4 * 2 + 1, k)] ...
        + mean([slackGps1(:, k), slackGps2(:, k), slackGps3(:, k), slackGps4(:, k)], 2) + slackmeanGPS(:, k));
end

% Rotation matrix for the final step
R = [cos(y_meas(4 * 2 + 1, Nh + 1)), -sin(y_meas(4 * 2 + 1, Nh + 1)), 0;
     sin(y_meas(4 * 2 + 1, Nh + 1)),  cos(y_meas(4 * 2 + 1, Nh + 1)), 0;
     0,                               0,                               0];

% Slack variables for GPS measurements at final step
optiMHE.subject_to(qMHE(:, Nh + 1) + vMHE(:, Nh + 1) == [y_meas(1:2, Nh + 1); y_meas(4 * 2 + 1, Nh + 1)] ...
    - R * 0.5 * [lg; -wg; 0] + slackGps1(:, Nh + 1));
optiMHE.subject_to(qMHE(:, Nh + 1) + vMHE(:, Nh + 1) == [y_meas(3:4, Nh + 1); y_meas(4 * 2 + 1, Nh + 1)] ...
    - R * 0.5 * [lg; wg; 0] + slackGps2(:, Nh + 1));
optiMHE.subject_to(qMHE(:, Nh + 1) + vMHE(:, Nh + 1) == [y_meas(5:6, Nh + 1); y_meas(4 * 2 + 1, Nh + 1)] ...
    - R * 0.5 * [-lg; wg; 0] + slackGps3(:, Nh + 1));
optiMHE.subject_to(qMHE(:, Nh + 1) + vMHE(:, Nh + 1) == [y_meas(7:8, Nh + 1); y_meas(4 * 2 + 1, Nh + 1)] ...
    - R * 0.5 * [-lg; -wg; 0] + slackGps4(:, Nh + 1));

% Slack variable for mean GPS measurements at final step
optiMHE.subject_to(qMHE(:, Nh + 1) + vMHE(:, Nh + 1) == [mean([y_meas(1:2, Nh + 1), y_meas(3:4, Nh + 1), y_meas(5:6, Nh + 1), y_meas(7:8, Nh + 1)], 2); y_meas(4 * 2 + 1, Nh + 1)] ...
    + mean([slackGps1(:, Nh + 1), slackGps2(:, Nh + 1), slackGps3(:, Nh + 1), slackGps4(:, Nh + 1)], 2) + slackmeanGPS(:, Nh + 1));

Jmhe = Jmhe + vMHE(:, Nh + 1).' * Rmhe * vMHE(:, Nh + 1);

% Set objective
optiMHE.minimize(Jmhe);

% Bounds on process and measurement noise
optiMHE.subject_to((-0.4 * 5 <= wMHE) <= 0.4 * 5);
optiMHE.subject_to((-0.4 * 5 <= vMHE) <= 0.4 * 5);

% *************************************************************************
% Solver Options
% *************************************************************************
p_optsMHE = struct('expand', true);
s_optsMHE = struct('sb', 'yes', 'print_level', 0, 'gamma_theta', 1e-2, 'jacobian_approximation', 'exact', 'fast_step_computation', 'yes', 'warm_start_init_point', 'yes');
optiMHE.solver('ipopt', p_optsMHE, s_optsMHE);

% *************************************************************************
% Define Path to Follow
% *************************************************************************
t = 0:0.01:2 * pi ;
x_lim = 5;
y_lim = 5;
x_path = x_lim * cos(t);
y_path = y_lim * sin(t);
thetaref = unwrap(atan2(diff(y_path), diff(x_path)));
qr = [x_path; y_path; [thetaref, thetaref(end)]];  % First reference

% *************************************************************************
% Lemniscate Path
% *************************************************************************
a = 5;  % Scale of the lemniscate
x_path = a * cos(t) ./ (1 + sin(t).^2);
y_path = a * sin(t) .* cos(t) ./ (1 + sin(t).^2);
thetaref = unwrap(atan2(diff(y_path), diff(x_path)));
qr = [x_path; y_path; [thetaref, thetaref(end)]];  % First reference

% *************************************************************************
% Square Path
% *************************************************************************
side_length = 5;                % Length of each side of the square
num_points = length(t);         % Number of points based on t
half_side = side_length / 2;
x_corners = [-half_side, half_side, half_side, -half_side, -half_side];
y_corners = [-half_side, -half_side, half_side, half_side, -half_side];

% Interpolate points along each side
x_path = [];
y_path = [];
points_per_side = num_points / 4;
for i = 1:length(x_corners) - 1
    x_path = [x_path, linspace(x_corners(i), x_corners(i + 1), points_per_side)];
    y_path = [y_path, linspace(y_corners(i), y_corners(i + 1), points_per_side)];
end

% Calculate the reference orientation (thetaref)
thetaref = unwrap(atan2(diff(y_path), diff(x_path)));

% Ensure the difference between angles is not greater than 2 * pi
angles_qr = unique(thetaref, 'stable');
for i = 2:length(angles_qr)
    if angles_qr(i) - angles_qr(i - 1) > pi
        angles_qr(i) = angles_qr(i) - 2 * pi;
    elseif angles_qr(i) - angles_qr(i - 1) < -pi
        angles_qr(i) = angles_qr(i) + 2 * pi;
    end
end

% Replace different values of thetaref with unique values
for i = 1:4
    idx_start = 1 + round(length(thetaref) / 4) * (i - 1);
    idx_end = round(length(thetaref) / 4) * i - 1;
    thetaref(idx_start:idx_end) = angles_qr(i);
end

% *************************************************************************
% Initialization
% *************************************************************************
optiMPC.set_value(u_last, [0; 0]);  % Initialize MPC parameter

q0 = [-2, 1, 1];  % Initial condition
R = [cos(q0(3)), -sin(q0(3)); sin(q0(3)), cos(q0(3))];
gps0_ = R * 0.5 * [lg, lg, -lg, -lg; -wg, wg, wg, -wg] + repmat(q0(1:2)', 1, 4);  % GPS measurements
gps0Bar = [gps0_(:) + gps_noise_factor .* randn(size(gps0_(:))); q0(3) + gyro_noise_factor .* randn(1)];  % First measurement
gps0Bar_mean = [mean(gps0Bar(1:2:end - 1)); mean(gps0Bar(2:2:end - 1)); gps0Bar(end)];

Q = zeros(nq, length(t));  % Real states of the system
Q(:, 1) = q0;
Qr = zeros(3, length(t));
Qe = zeros(3, length(t));
Qestimated = zeros(nq, length(t));
Qmean = zeros(nq, length(t));
u_mpc = zeros(nu, length(t));
u_kc = zeros(nu, length(t));
optiMPC.set_value(u_last, [0; 0]);

% *************************************************************************
% Simulation
% *************************************************************************
% Initialization
optiMHE.set_value(uPast, repmat([0; 0], 1, Nh));  % Initialize past control inputs
optiMHE.set_value(xBar, gps0Bar_mean(:));         % Set initial state by the mean of the GPS measurements
optiMHE.set_value(y_meas, repmat(gps0Bar(:), 1, Nh + 1));
optiMHE.set_value(uPast, repmat([0; 0], 1, Nh));
solutionMHE = optiMHE.solve();

% *************************************************
% LPF Initialization
% *************************************************
LPF_size = 18;  % Size of the LPF
LPF_buff = repmat(gps0Bar_mean, 1, LPF_size);  % Buffer for the LPF
Qlpf_estimated = zeros(nq, length(t));  % Estimated states by LPF

% Simulation loop
elapsed_time = zeros(1, length(qr));

% Initialize figure and legends
figure(1);
hold on;
grid on;
title('Trajectory Tracking');
xlim([-x_lim * 1.1, x_lim * 1.1]);
ylim([-y_lim * 1.1, y_lim * 1.1]);
xlabel('X');
ylabel('Y');
h1 = plot(NaN, NaN, 'k', 'DisplayName', 'Reference Trajectory', 'LineWidth', 1.5);
h2 = plot(NaN, NaN, 'b', 'DisplayName', 'Actual Position', 'LineWidth', 1.5);
h3 = plot(NaN, NaN, 'r--', 'DisplayName', 'Estimated Position', 'LineWidth', 1.5);
h4 = plot(NaN, NaN, 'g.', 'DisplayName', 'Mean GPS', 'LineWidth', 1.5);
h5 = plot(NaN, NaN, 'm.', 'DisplayName', 'Kalman Filter', 'LineWidth', 1.5);

% Initialize orientation triangle handle
h_triangle = [];

legend([h1, h2, h3, h5]);

% Initialize Extended Kalman Filter (EKF)
% Define process and measurement noise matrices
Qk = 0.01 * eye(3);  % Process noise covariance
Rk = diag([gps_noise_factor, gps_noise_factor, gyro_noise_factor]) * 0.5;  % Measurement noise covariance

% Initialize EKF state and covariance
x_kalman = gps0Bar_mean;  % Initial state
P_kalman = Rk;            % Initial covariance

% Initialize Qkalman
Qkalman = zeros(size(Q));

for i = 1:length(qr)
    % Measure execution time
    tic;
    % MHE
    % Update MHE
    qTrajEstimated = solutionMHE.value(qMHE);    % Get estimated state
    yTrajEstimated = solutionMHE.value(y_meas);  % Get estimated measurements
    uTrajEstimated = solutionMHE.value(uPast);   % Get estimated control inputs

    optiMHE.set_value(xBar, qTrajEstimated(:, 2));  % Set initial state

    % GPS measurements
    last_gps = repmat(Q(1:2, i), 1, 4) + [cos(Q(3, i)), -sin(Q(3, i)); sin(Q(3, i)), cos(Q(3, i))] * 0.5 * [lg, lg, -lg, -lg; -wg, wg, wg, -wg];
    last_measurement = [last_gps(:) + gps_noise_factor .* randn(size(last_gps(:))); Q(3, i) + gyro_noise_factor .* randn(1)];
    Qmean(:, i) = [mean(last_measurement(1:2:end - 1)); mean(last_measurement(2:2:end - 1)); last_measurement(end)];

    % Update MHE
    optiMHE.set_value(y_meas, [yTrajEstimated(:, 2:end), last_measurement]);
    optiMHE.set_value(uPast, [uTrajEstimated(:, 2:end), u_mpc(:, i)]);
    solutionMHE = optiMHE.solve();  % Solve MHE
    optiMHE.set_initial(solutionMHE.value_variables);  % Update initial condition
    Qaux = solutionMHE.value(qMHE);  % Get estimated state
    Qestimated(:, i) = Qaux(:, end);  % Get final estimated state

    % Update LPF
    LPF_buff = [LPF_buff(:, 2:end), Qmean(:, i)];
    Qlpf_estimated(:, i) = mean(LPF_buff, 2);

    if i < length(qr)
        % MPC
        optiMPC.set_value(qref, qr(:, i));           % Set reference
        optiMPC.set_value(qinit, Qestimated(:, i));  % Set initial state
        try
            solutionMPC = optiMPC.solve();  % Solve MPC
            optiMPC.set_initial(solutionMPC.value_variables());  % Update initial condition
            u_mpc(:, i + 1) = solutionMPC.value(u(:, 1));  % Get control input
            optiMPC.set_value(u_last, u_mpc(:, i + 1));    % Update last control input
        catch
            disp('MPC Solver failed');
            u_mpc(:, i + 1) = u_mpc(:, i);  % Use previous control input
        end

        % Measure execution time
        elapsed_time(i) = toc;

        % Extended Kalman Filter (EKF)
        % State prediction
        x_pred = F(x_kalman, u_mpc(:, i), Ts, w, zeros(3, 1));

        % Calculate Jacobian of the state transition function
        theta = x_kalman(3);
        v = (u_mpc(1, i) + u_mpc(2, i)) / 2;

        F_jacobian = [
            1, 0, -Ts * v * sin(theta);
            0, 1,  Ts * v * cos(theta);
            0, 0, 1
        ];

        % Covariance prediction
        P_pred = F_jacobian * P_kalman * F_jacobian' + Qk;

        % Update
        H_jacobian = eye(3);  % Jacobian of the observation function

        % Innovation
        y_tilde = Qmean(:, i) - x_pred;

        % Kalman gain
        S = H_jacobian * P_pred * H_jacobian' + Rk;
        K = P_pred * H_jacobian' / S;

        % State and covariance update
        x_kalman = x_pred + K * y_tilde;
        P_kalman = (eye(3) - K * H_jacobian) * P_pred;

        % Save estimated state by EKF
        Qkalman(:, i) = x_kalman;

        % System evolution
        if i == round(length(qr) / 3)
            Q(:, i) = Q(:, i) + [-1; 3; pi];  % Alter state to simulate perturbation
        end
        Q(:, i + 1) = F(Q(:, i), u_mpc(:, i), Ts, w, zeros(3, 1));  % Evolve the system
    end

    % Measure execution time
    disp('Max elapsed time');
    disp((elapsed_time(i)));

    % Update plots
    cla;
    plot(qr(1, 1:i), qr(2, 1:i), 'k', 'DisplayName', 'Reference Trajectory', 'LineWidth', 1.5);
    plot(Q(1, 1:i), Q(2, 1:i), 'b', 'DisplayName', 'Actual Position', 'LineWidth', 1.5);
    plot(Qestimated(1, 1:i), Qestimated(2, 1:i), 'r--', 'DisplayName', 'MHE Position', 'LineWidth', 1.5);
    plot(Qkalman(1, 1:i), Qkalman(2, 1:i), 'm.', 'DisplayName', 'EKF Position', 'LineWidth', 1.5);

    % Plot GPS measurements
    plot(last_measurement(1:2:end - 1), last_measurement(2:2:end - 1), 'ro', 'DisplayName', 'GPS noisy measurements');

    % Update orientation triangle
    if ~isempty(h_triangle)
        delete(h_triangle);
    end
    head_width = 0.3;   % Width of the arrowhead
    head_length = head_width;  % Length of the arrowhead
    theta = Q(3, i);    % Vehicle orientation

    % Coordinates of the triangle in the vehicle's reference frame
    x_triangle = [0, -head_length, -head_length];
    y_triangle = [0, head_width / 2, -head_width / 2];

    % Rotate and translate the triangle to the vehicle's position and orientation
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    triangle = R * [x_triangle + head_length * 0.5; y_triangle];
    h_triangle = fill(Q(1, i) + triangle(1, :), Q(2, i) + triangle(2, :), 'b', 'EdgeColor', 'none', 'HandleVisibility', 'off');

    drawnow;
end

% ******************************************************************
% Plot Results
% ******************************************************************
figure(2);
hold on;
grid on;
plot(t, u_mpc(1, :), 'Color', [0.3010, 0.7450, 0.9330], 'DisplayName', 'MPC Control Input 1 VR');
plot(t, u_mpc(2, :), 'Color', [0.8500, 0.3250, 0.0980], 'DisplayName', 'MPC Control Input 2 VL');
legend('show');
title('Control Inputs');
xlabel('Time Step');
ylabel('Control Input');

figure(3);
% First subplot on the left
subplot(1, 2, 1);
hold on;
grid on;
plot(t, sqrt((Qestimated(1, :) - Q(1, :)).^2 + (Qestimated(2, :) - Q(2, :)).^2), 'r', 'DisplayName', 'Norm position error MHE');
plot(t, sqrt((Qkalman(1, :) - Q(1, :)).^2 + (Qkalman(2, :) - Q(2, :)).^2), 'm', 'DisplayName', 'Norm position error EKF');
legend('show');
title('Norm Position Error');
xlabel('Time Step');
ylabel('Norm Position Error');

% Second subplot on the right
subplot(1, 2, 2);
hold on;
grid on;
plot(t, abs(Qestimated(3, :) - Q(3, :)), 'r', 'DisplayName', 'Error dir MHE');
plot(t, abs(Qkalman(3, :) - Q(3, :)), 'm', 'DisplayName', 'Error dir EKF');
legend('show');
title('Error in Direction');
xlabel('Time Step');
ylabel('Error Direction');

% Plot execution time
figure(4);
hold on;
grid on;
plot(elapsed_time, 'b', 'DisplayName', 'Elapsed time by iteration');
legend('show');
title('Elapsed time by iteration');
xlabel('Iteration');
ylabel('Elapsed time in seconds');

% Plot position vs. reference error
figure(5);
% First subplot on the left
subplot(1, 2, 1);
hold on;
grid on;
plot(sqrt((Q(1, :) - qr(1, :)).^2 + (Q(2, :) - qr(2, :)).^2), 'b', 'DisplayName', 'Norm position error Q');
legend('show');
title('Norm Position Error vs Reference');
xlabel('Time Step');
ylabel('Norm Position Error');

% Second subplot on the right
subplot(1, 2, 2);
hold on;
grid on;
plot(Q(3, :) - qr(3, :), 'b', 'DisplayName', 'Error dir Q');
legend('show');
title('Error in Direction vs Reference');
xlabel('Time Step');
ylabel('Error Direction');

% Display RMS error
disp('RMS error in position vs kalman');
disp(rms(sqrt((Qkalman(1, :) - Q(1, :)).^2 + (Qkalman(2, :) - Q(2, :)).^2)) - rms(sqrt((Qestimated(1, :) - Q(1, :)).^2 + (Qestimated(2, :) - Q(2, :)).^2)));

disp('RMS error in direction vs kalman');
disp(rms(abs(Qkalman(3, :) - Q(3, :))) - rms(abs(Qestimated(3, :) - Q(3, :))));

disp('Max error in position estimated percentage compared to noise');
disp(max(sqrt((Qkalman(1, :) - Q(1, :)).^2 + (Qkalman(2, :) - Q(2, :)).^2)) / gps_noise_factor);

% Dynamics function
function Fk = F(q, u, Ts, w, noise)
    Fk = q + Ts * [cos(q(3)), 0;
                   sin(q(3)), 0;
                   0,        1] * [1/2, 1/2; 1/w, -1/w] * u + noise;
end