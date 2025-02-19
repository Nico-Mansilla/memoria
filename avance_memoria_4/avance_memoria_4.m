% MPC and MHE implementation for a differential drive robot
% EKF comparison
% EKF arrvial cost is used as MHE arrival cost
% ------------------------------------------------------------------------

clearvars
close all
clc

addpath(fullfile(pwd, 'casadi-3.6.5-windows64-matlab2018b'))
import casadi.*

% ------------------------------------------------------------------------
% General Parameters:
% ------------------------------------------------------------------------
Ts = 0.1;  % Sampling time sensors in s
sampling_factor=1000; % Sampling factor for simulation
sim_Ts = Ts/sampling_factor;  % Simulation sampling time

gps1_noise_factor= 0.1*1;
gps2_noise_factor= 0.1*2;
gps3_noise_factor= 0.1*4;
gps4_noise_factor= 0.1*8;

gps_noise_factor = sqrt((gps1_noise_factor^2+gps2_noise_factor^2+gps3_noise_factor^2+gps4_noise_factor^2)/16);  % GPS noise factor standard deviation
gps_noise_factor_variance = gps_noise_factor^2;  % GPS noise factor variance

gyro_noise_factor = 2 * pi * 0.005;  % Gyro noise factor standard deviation
gyro_noise_factor_variance = gyro_noise_factor^2;  % Gyro noise factor variance

% -------------------------------------------------------------------------
% Vehicle's Geometrical Properties:
% -------------------------------------------------------------------------

w = 0.555;  % Robot width (between wheels)
l = 0.544;  % Robot length

wg = 0.6;  % GPS width
lg = 0.6;  % GPS length

vel_max = 4;  % Maximum tangent velocity in m/s

wheel_radius = 0.1;  % Wheel radius in meters

% -------------------------------------------------------------------------
% System's Properties:
% -------------------------------------------------------------------------

nq = 3;  % Number of states (x, y, theta)
nu = 2;  % Number of inputs (vr, vl)
nr = 3;  % Number of reference states

% -------------------------------------------------------------------------
% Model Predictive Control (MPC) Parameters:
% -------------------------------------------------------------------------
Nc = 30;  % Control horizon length

max_angular_velocity = vel_max / wheel_radius;% Maximum angular velocity in rad/s
wheel_vel = max_angular_velocity/2; % Maximum wheel velocity in rad/s

U_lb = [-wheel_vel; -wheel_vel];  % Lower bound for the control inputs
U_ub = [wheel_vel; wheel_vel];    % Upper bound for the control inputs

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

% -------------------------------------------------------------------------
% MPC Objective Function
% -------------------------------------------------------------------------
Jmpc = 0;
for k = 1:Nc + 1
    % Error between predicted state and reference state
    error = qMPC(:, k) - qref;
    % Adjust the angle to be within the range [-pi, pi]
    for i = 1:6
        error(3) = if_else(error(3) > pi, error(3) - 2 * pi, error(3));
        error(3) = if_else(error(3) < -pi, error(3) + 2 * pi, error(3));
    end

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

% -------------------------------------------------------------------------
% Moving Horizon Estimation (MHE) Parameters:
% -------------------------------------------------------------------------
Nh = 20;  % MHE horizon length
optiMHE = casadi.Opti();

qMHE = optiMHE.variable(nq, Nh + 1);        % State estimates
wMHE = optiMHE.variable(nq, Nh);            % Process noise
vMHE = optiMHE.variable(nq, Nh + 1);        % Measurement noise

alphaMHE = optiMHE.variable(4, Nh+1);       % Convex sum of the GPS measurements


y_meas = optiMHE.parameter(4 * 2 + 1, Nh + 1);  % Measurements
y_meas_convx = optiMHE.variable(3, Nh + 1);  % Convex sum of the GPS measurements

uPast = optiMHE.parameter(nu, Nh);          % Past control inputs applied to the system
X = optiMHE.variable(nq);                   % Arrival-cost
xBar = optiMHE.parameter(nq);               % Initial state for MHE
Pmhe = optiMHE.parameter(nq,nq);                % Arrival cost
JmhePast = optiMHE.parameter(1);             % Arrival cost

% Cost matrices for MHE
Rmhe = diag([1/gps_noise_factor_variance, 1/gps_noise_factor_variance, 1/gyro_noise_factor_variance])*0.48;  % Measurement cost (greater means less confidence)
Qmhe = diag([1, 1, 1/(2*pi)] * 1400);            % Process noise cost

% -------------------------------------------------------------------------
% MHE Objective Function
% -------------------------------------------------------------------------
% Set initial state constraint

Jmhe = X.' * Pmhe * X + JmhePast;  % Arrival cost
optiMHE.subject_to(X == qMHE(:, 1) - xBar);

for k = 1:Nh
    Jmhe = Jmhe + vMHE(:, k).' * Rmhe * vMHE(:, k) + wMHE(:, k).' * Qmhe * wMHE(:, k);

    optiMHE.subject_to(qMHE(:, k + 1) == F(qMHE(:, k), uPast(:, k), Ts, w, wMHE(:, k)));

    % Rotation matrix
    R = [cos(y_meas(4 * 2 + 1, k)), -sin(y_meas(4 * 2 + 1, k));
         sin(y_meas(4 * 2 + 1, k)),  cos(y_meas(4 * 2 + 1, k))];

    %Convex sum of the GPS measurements
    
    optiMHE.subject_to(sum(alphaMHE(:,k))==1);
    optiMHE.subject_to((0<=alphaMHE(:,k))<=1);

    y_meas_convx (:,k)= [(y_meas(1:2, k) - R * 0.5 * [lg; -wg]) * alphaMHE(1, k) + ...
                        (y_meas(3:4, k) - R * 0.5 * [lg; wg]) * alphaMHE(2, k) + ...
                        (y_meas(5:6, k) - R * 0.5 * [-lg; wg]) * alphaMHE(3, k) + ...
                        (y_meas(7:8, k) - R * 0.5 * [-lg; -wg]) * alphaMHE(4, k); y_meas(4 * 2 + 1, k)];

    
    optiMHE.subject_to(qMHE(:, k) + vMHE(:, k) == (y_meas_convx (:,k)));
    
end

% Rotation matrix for the final step
R = [cos(y_meas(4 * 2 + 1, Nh + 1)), -sin(y_meas(4 * 2 + 1, Nh + 1));
     sin(y_meas(4 * 2 + 1, Nh + 1)),  cos(y_meas(4 * 2 + 1, Nh + 1))];      


%Convex sum of the GPS measurements

optiMHE.subject_to(sum(alphaMHE(:,Nh+1))==1);
optiMHE.subject_to((0<=alphaMHE(:,Nh+1))<=1);

y_meas_convx(:,k) = [(y_meas(1:2, Nh + 1) - R * 0.5 * [lg; -wg]) * alphaMHE(1, Nh + 1) + ...
                (y_meas(3:4, Nh + 1) - R * 0.5 * [lg; wg]) * alphaMHE(2, Nh + 1) + ...
                (y_meas(5:6, Nh + 1) - R * 0.5 * [-lg; wg]) * alphaMHE(3, Nh + 1) + ...
                (y_meas(7:8, Nh + 1) - R * 0.5 * [-lg; -wg]) * alphaMHE(4, Nh + 1); y_meas(4 * 2 + 1, Nh + 1)];

optiMHE.subject_to(qMHE(:, Nh + 1) + vMHE(:, Nh + 1) == y_meas_convx(:,Nh+1) );

Jmhe = Jmhe + vMHE(:, Nh + 1).' * Rmhe * vMHE(:, Nh + 1);



% Set objective
optiMHE.minimize(Jmhe);

% Bounds on process and measurement noise
optiMHE.subject_to((-gps_noise_factor*3 <= wMHE) <= gps_noise_factor*3 );
optiMHE.subject_to((-gps_noise_factor*3 <= vMHE) <= gps_noise_factor*3 );

% -------------------------------------------------------------------------
% Solver Options
% -------------------------------------------------------------------------
p_optsMHE = struct('expand', true);
s_optsMHE = struct('sb', 'yes', 'print_level', 0, 'gamma_theta', 1e-2, 'jacobian_approximation', 'exact', 'fast_step_computation', 'yes', 'warm_start_init_point', 'yes');
optiMHE.solver('ipopt', p_optsMHE, s_optsMHE);

% -------------------------------------------------------------------------
% Define Path to Follow
% -------------------------------------------------------------------------

cicles = 2;  % Number of cicles to complete

ref_samples = 0: sim_Ts :2 * pi * cicles;  % Reference samples


% -------------------------------------------------------------------------
% Lemniscate Path
% -------------------------------------------------------------------------
scale = 6;  % Scale of the lemniscate in meters

x_lim = scale;
y_lim = scale;

x_path = scale * cos(ref_samples) ./ (1 + sin(ref_samples).^2);
y_path = scale * sin(ref_samples) .* cos(ref_samples) ./ (1 + sin(ref_samples).^2);
thetaref = unwrap(atan2(diff(y_path), diff(x_path)));

qr = [x_path; y_path; [thetaref, thetaref(end)]];  % First reference

distance = sqrt(diff(x_path).^2 + diff(y_path).^2); % Distance between points in meters
total_distance = sum(distance); %total distance of the path

vref = 0.9;  % Velocity of the reference in m/s

total_time = total_distance / vref;  % Total time to complete the path



t = 0: Ts : total_time;  % Time vector

% Calculate the number of samples needed
num_samples = length(t);

% Downsample qr to match the length of t
downsampling_factor = floor(size(qr, 2) / num_samples);
qr_downsampled = qr(:, 1:downsampling_factor:end);

% Ensure the last point is included
if size(qr_downsampled, 2) < num_samples
    qr_downsampled = [qr_downsampled, qr(:, end)];
end

% Update qr with the downsampled version
qr = qr_downsampled;

% Recalculate t to have the same number of samples as qr
num_samples = size(qr, 2);
t = linspace(0, total_time, num_samples);

% -------------------------------------------------------------------------
% Initialization
% -------------------------------------------------------------------------


optiMPC.set_value(u_last, [0; 0]);  % Initialize MPC parameter

q0 = [-2, 1, 1];  % Initial condition
R = [cos(q0(3)), -sin(q0(3)); sin(q0(3)), cos(q0(3))];
gps0_ = R * 0.5 * [lg, lg, -lg, -lg; -wg, wg, wg, -wg] + repmat(q0(1:2)', 1, 4);  % GPS measurements
gps0Bar = [gps0_(:) + [gps1_noise_factor .* randn(2,1); gps2_noise_factor .* randn(2,1); gps3_noise_factor .* randn(2,1); gps4_noise_factor .* randn(2,1)]; q0(3) + gyro_noise_factor .* randn(1)]+scale*[repmat([0.2;0.2],4,1);pi*0.1];
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

% -------------------------------------------------------------------------
% Simulation
% -------------------------------------------------------------------------
% Initialization
optiMHE.set_value(uPast, repmat([0; 0], 1, Nh));  % Initialize past control inputs
optiMHE.set_value(xBar, gps0Bar_mean(:));         % Set initial state by the mean of the GPS measurements
optiMHE.set_value(JmhePast, 0);                    % Set initial arrival cost
optiMHE.set_value(Pmhe, diag([1/gps_noise_factor_variance, 1/gps_noise_factor_variance, 1/gyro_noise_factor_variance]));  % Set initial covariance
optiMHE.set_value(y_meas, repmat(gps0Bar(:), 1, Nh + 1));
optiMHE.set_value(uPast, repmat([0; 0], 1, Nh));
optiMHE.set_initial(alphaMHE, repmat([0.25; 0.25; 0.25; 0.25], 1, Nh + 1)); % Initialize convex sum of the GPS measurements
solutionMHE = optiMHE.solve();


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
h5 = plot(NaN, NaN, 'm.', 'DisplayName', 'Kalman Filter', 'LineWidth', 1.5);
% Initialize orientation triangle handle
h_triangle = [];

legend([h1, h2, h3, h5]);

% Initialize Extended Kalman Filter (EKF)
% Define process and measurement noise matrices
Qk = 0.00001 * eye(3);  % Process noise covariance
Rk = diag([gps_noise_factor_variance, gps_noise_factor_variance, gyro_noise_factor_variance]);  % Measurement noise covariance

% Initialize EKF state and covariance
x_kalman = gps0Bar_mean;  % Initial state
P_kalman = Rk;            % Initial covariance

% Initialize Qkalman
Qkalman = zeros(size(Q));

for i = 1:length(qr)

    % GPS measurements
    last_gps = repmat(Q(1:2, i), 1, 4) + [cos(Q(3, i)), -sin(Q(3, i)); sin(Q(3, i)), cos(Q(3, i))] * 0.5 * [lg, lg, -lg, -lg; -wg, wg, wg, -wg];
    last_measurement = [last_gps(:) + [gps1_noise_factor .* randn(2,1); gps2_noise_factor .* randn(2,1); gps3_noise_factor .* randn(2,1); gps4_noise_factor .* randn(2,1)]; Q(3, i) + gyro_noise_factor .* randn(1)];
    Qmean(:, i) = [mean(last_measurement(1:2:end - 1)); mean(last_measurement(2:2:end - 1)); last_measurement(end)];

    % Measure execution time
    tic;
    
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
    
    % Update Pmhe
    P_kalman_inv=inv(P_kalman);
    optiMHE.set_value(Pmhe, P_kalman_inv);

    
    % MHE
    % Update MHE
    qTrajEstimated = solutionMHE.value(qMHE);    % Get estimated state
    yTrajEstimated = solutionMHE.value(y_meas);  % Get estimated measurements
    uTrajEstimated = solutionMHE.value(uPast);   % Get estimated control inputs

    optiMHE.set_value(xBar, qTrajEstimated(:, 2));  % Set initial state

    
    % Update MHE
    optiMHE.set_value(y_meas, [yTrajEstimated(:, 2:end), last_measurement]);
    optiMHE.set_value(uPast, [uTrajEstimated(:, 2:end), u_mpc(:, i)]);
    solutionMHE = optiMHE.solve();  % Solve MHE
    optiMHE.set_initial(solutionMHE.value_variables);  % Update initial condition
    optiMHE.set_value(JmhePast, solutionMHE.value(Jmhe));  % Update arrival cost    
    Qaux = solutionMHE.value(qMHE);  % Get estimated state
    Qestimated(:, i) = Qaux(:, end);  % Get final estimated state



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


        % System evolution
        if i == round(length(qr) / 3)
            %Q(:, i) = Q(:, i) + [-1; 3; pi];  % Alter state to simulate perturbation
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
    plot(solutionMHE.value(y_meas_convx(1,end-1)), solutionMHE.value(y_meas_convx(2,end-1)), 'r*', 'DisplayName', 'GPS estimated measurements');


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
    triangle = R * [x_triangle + head_length * 0.5; y_triangle]*scale/4;
    h_triangle = fill(Q(1, i) + triangle(1, :), Q(2, i) + triangle(2, :), 'b', 'EdgeColor', 'none', 'HandleVisibility', 'off');

    drawnow;
end

% ------------------------------------------------------------------
% Plot Results
% ------------------------------------------------------------------
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
plot(t(1:end-1), sqrt((Qestimated(1, 1:end-1) - Q(1, 1:end-1)).^2 + (Qestimated(2, 1:end-1) - Q(2, 1:end-1)).^2), 'r', 'DisplayName', 'Norm position error MHE');
plot(t(1:end-1), sqrt((Qkalman(1, 1:end-1) - Q(1, 1:end-1)).^2 + (Qkalman(2, 1:end-1) - Q(2, 1:end-1)).^2), 'm', 'DisplayName', 'Norm position error EKF');
legend('show');
title('Norm Position Error');
xlabel('Time Step');
ylabel('Norm Position Error');

% Second subplot on the right
subplot(1, 2, 2);
hold on;
grid on;
plot(t(1:end-1), abs(Qestimated(3, 1:end-1) - Q(3, 1:end-1)), 'r', 'DisplayName', 'Error dir MHE');
plot(t(1:end-1), abs(Qkalman(3, 1:end-1) - Q(3, 1:end-1)), 'm', 'DisplayName', 'Error dir EKF');
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

disp('Mean error diff in position EKF-MHE:');
disp(sqrt(mean((Qkalman(1, 1:end-1) - Q(1, 1:end-1)).^2 + (Qkalman(2, 1:end-1) - Q(2, 1:end-1)).^2))-mean(sqrt((Qestimated(1, 1:end-1) - Q(1, 1:end-1)).^2 + (Qestimated(2, 1:end-1) - Q(2, 1:end-1)).^2)));

disp('Mean error diff in direction EKF-MHE:');
disp(mean(abs(Qkalman(3, 1:end-1) - Q(3, 1:end-1)))-mean(abs(Qestimated(3, 1:end-1) - Q(3, 1:end-1))));

disp('Mean error estimation MHE:');
disp(mean(sqrt((Q(1, :) - Qestimated(1, :)).^2 + (Q(2, :) - Qestimated(2, :)).^2)));

% Dynamics function
function Fk = F(q, u, Ts, w, noise)
    Fk = q + Ts * [cos(q(3)), 0;
                   sin(q(3)), 0;
                   0,        1] * [1/2, 1/2; 1/w, -1/w] * u + noise;
end