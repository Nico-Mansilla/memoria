%MHE Funcional comparado con mean LPF
% 
% 
% ************************************************************************

clearvars
close all
clc

addpath('C:\Users\nicolas\Desktop\Memoria\casadi-3.6.5-windows64-matlab2018b')
import casadi.*

% *************************************************************************
% General Parameters:
% *************************************************************************
Ts = 0.100;  % Sampling time
gps_noise_factor=0.1; % Noise factor
gyro_noise_factor = 2*pi*0.005; % Noise factor

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
U_ub = [4; 4];  % Upper bound for the control inputs

dU_lb = U_lb*0.1;  % Lower bound for the acceleration
dU_ub = U_ub*0.1;    % Upper bound for the acceleration


% Create Casadi optimization object
optiMPC = casadi.Opti();

% Define MPC variables
qMPC = optiMPC.variable(nq, Nc+1);  % Predicted state trajectories
u = optiMPC.variable(nu, Nc);       % Control inputs
qref = optiMPC.parameter(nr);       % Reference state
qinit = optiMPC.parameter(nq);      % Initial state
u_last = optiMPC.parameter(nu);     % Last control input

% Cost matrices
Wmpc = diag([1 1 0.1]);    % State cost weight matrix
Rmpc = diag([1*0.001 1*0.001]);  % Control input cost weight matrix

% *************************************************************************
% MPC Objective Function
% *************************************************************************
Jmpc = 0;
for k = 1:Nc+1
    Jmpc = Jmpc + (qMPC(:,k) - qref).' * Wmpc * (qMPC(:,k) - qref);
    if k < Nc+1
        optiMPC.subject_to(qMPC(:,k+1) == F(qMPC(:,k), u(:,k), Ts, w, zeros(3,1)));
        
        %acceleration constraints
        if k ~= 1
            optiMPC.subject_to((dU_lb <= (u(:,k) - u(:,k-1))/Ts) <= dU_ub);
        end
        optiMPC.subject_to((U_lb <= u(:,k)) <= U_ub);
        Jmpc = Jmpc + u(:,k).' * Rmpc * u(:,k);
    end
end

% Set objective
optiMPC.minimize(Jmpc);

% Set initial state constraint
optiMPC.subject_to(qMPC(:,1) == qinit);

% Solver options
p_optsMPC = struct('expand', true);
s_optsMPC = struct('sb', 'yes', 'print_level', 0, 'warm_start_init_point', 'yes');
optiMPC.solver('ipopt', p_optsMPC, s_optsMPC);

% *************************************************************************
% Moving Horizon Estimation (MHE) Parameters:
% *************************************************************************
Nh = 20;  % MHE horizon length
optiMHE = casadi.Opti();

qMHE = optiMHE.variable(nq, Nh+1);  % State estimates
wMHE = optiMHE.variable(nq, Nh);       % Process noise
vMHE = optiMHE.variable(nq, Nh+1);       % Measurement noise

slackGps1 = optiMHE.variable(nq, Nh+1); % Slack variables for GPS measurements
slackGps2 = optiMHE.variable(nq, Nh+1); % Slack variables for GPS measurements
slackGps3 = optiMHE.variable(nq, Nh+1); % Slack variables for GPS measurements
slackGps4 = optiMHE.variable(nq, Nh+1); % Slack variables for GPS measurements


y_meas = optiMHE.parameter(4*2+1, Nh+1);     % Measurements


uPast = optiMHE.parameter(nu, Nh);     % Past control inputs applied to the system
X       = optiMHE.variable(nq);         % Arrival-cost
xBar    = optiMHE.parameter(nq);        % Initial state for MHE

% Cost matrices for MHE
Rmhe = diag([gps_noise_factor gps_noise_factor gyro_noise_factor]); %q pert
Qmhe = diag([1 1 1]*100); % measure pert
Pmhe = diag([gps_noise_factor gps_noise_factor 100]); %arribal cost

% *************************************************************************
% MHE Objective Function
% *************************************************************************
% Set initial state constraint

Jmhe = X.'*Pmhe*X; % ARRIVAL-COST
optiMHE.subject_to(X == qMHE(:,1)-xBar);

for k = 1:Nh
    Jmhe = Jmhe + vMHE(:,k).'*Rmhe*vMHE(:,k) + wMHE(:,k).'*Qmhe*wMHE(:,k);
    optiMHE.subject_to(qMHE(:,k+1) == F(qMHE(:,k), uPast(:,k), Ts, w, wMHE(:,k)));
    

    % Rotation matrix
    R = [cos(y_meas(4*2+1,k)), -sin(y_meas(4*2+1,k)), 0;
         sin(y_meas(4*2+1,k)),  cos(y_meas(4*2+1,k)), 0;
                            0,                     0, 0];

    % GPS measurements
    optiMHE.subject_to(qMHE(:,k) + vMHE(:,k) == [y_meas(1:2,k);y_meas(4*2+1,k)] - R*0.5*[lg ; -wg ; 0] + slackGps1(:,k)); 
    optiMHE.subject_to(qMHE(:,k) + vMHE(:,k) == [y_meas(3:4,k);y_meas(4*2+1,k)] - R*0.5*[lg ; wg ; 0] + slackGps2(:,k)); 
    optiMHE.subject_to(qMHE(:,k) + vMHE(:,k) == [y_meas(5:6,k);y_meas(4*2+1,k)] - R*0.5*[-lg ; wg ; 0] + slackGps3(:,k));
    optiMHE.subject_to(qMHE(:,k) + vMHE(:,k) == [y_meas(7:8,k);y_meas(4*2+1,k)] - R*0.5*[-lg ; -wg ; 0] + slackGps4(:,k));
end

% Rotation matrix
R = [cos(y_meas(4*2+1,Nh+1)), -sin(y_meas(4*2+1,Nh+1)), 0;
     sin(y_meas(4*2+1,Nh+1)),  cos(y_meas(4*2+1,Nh+1)), 0;
                           0,                        0, 0]; 

optiMHE.subject_to(qMHE(:,Nh+1) + vMHE(:,Nh+1) == [y_meas(1:2,Nh+1);y_meas(4*2+1,Nh+1)] - R*0.5*[lg ; -wg ; 0] + slackGps1(:,Nh+1));
optiMHE.subject_to(qMHE(:,Nh+1) + vMHE(:,Nh+1) == [y_meas(3:4,Nh+1);y_meas(4*2+1,Nh+1)] - R*0.5*[lg ; wg ; 0] + slackGps2(:,Nh+1)); 
optiMHE.subject_to(qMHE(:,Nh+1) + vMHE(:,Nh+1) == [y_meas(5:6,Nh+1);y_meas(4*2+1,Nh+1)] - R*0.5*[-lg ; wg ; 0] + slackGps3(:,Nh+1));
optiMHE.subject_to(qMHE(:,Nh+1) + vMHE(:,Nh+1) == [y_meas(7:8,Nh+1);y_meas(4*2+1,Nh+1)] - R*0.5*[-lg ; -wg ; 0] + slackGps4(:,Nh+1));

Jmhe = Jmhe + vMHE(:,Nh+1).'*Rmhe*vMHE(:,Nh+1);%+ wMHE(:,Nh+1).'*Qmhe*wMHE(:,Nh+1);

% Set objective
optiMHE.minimize(Jmhe);



% Bounds on process and measurement noise
optiMHE.subject_to((-0.4 <= wMHE) <= 0.4);
optiMHE.subject_to((-0.4 <= vMHE) <= 0.4);

% *************************************************************************

% Some parameters for the solver (do not care about that, for now...)
p_optsMHE = struct('expand',true);
s_optsMHE = struct('sb','yes','print_level',0,'gamma_theta',1e-2,'jacobian_approximation','exact','fast_step_computation','yes','warm_start_init_point','yes'); % 
optiMHE.solver('ipopt',p_optsMHE,s_optsMHE);


% *************************************************************************
% Define Path to Follow
% *************************************************************************
t           = 0:0.01:2*pi;
x_lim       = 5;
y_lim       = 5;
x_path      = x_lim*cos(t);
y_path      = y_lim*sin(t);
thetaref    = unwrap(atan2(diff(y_path),diff(x_path)));
qr          = [ x_path; y_path;[thetaref,thetaref(end)]];           % First reference. This reference need to be updated then in every sampling time
% *************************************************************************
%Lemniscata
a           = 5; % Escala de la lemniscata
x_path      = a * cos(t) ./ (1 + sin(t).^2);
y_path      = a * sin(t) .* cos(t) ./ (1 + sin(t).^2);
thetaref    = unwrap(atan2(diff(y_path), diff(x_path)));
qr          = [x_path; y_path; [thetaref, thetaref(end)]]; % First reference. This reference needs to be updated then in every sampling time
% *************************************************************************

optiMPC.set_value(u_last,[0;0]);                                % init this parameter of the MPC
%

q0          = [-2,1,1];    % This function generates a feasible initila condition: The vehicle is a challenging one
R = [cos(q0(3)), -sin(q0(3)); sin(q0(3)), cos(q0(3))];
gps0_        = R*0.5*[lg lg -lg -lg ; -wg wg wg -wg] + repmat(q0(1:2)',1,4); % GPS measurements
gps0Bar        = [gps0_(:)+ gps_noise_factor.*randn(size(gps0_(:))) ; q0(3)+gyro_noise_factor.*randn(1)]; %first measure

%q0Bar       = q0 + noise_factor.*randn(size(q0));  %first measure 
%gps0Bar     = gps0 + gps_noise_factor.*randn(size(gps0)); %first measure
gps0Bar_mean = [mean(gps0Bar(1:2:end-1)); mean(gps0Bar(2:2:end-1)); gps0Bar(end)];

Q           = zeros(nq,length(t));%real states of the system
Q(:,1)      = q0;
Qr          = zeros(3,length(t));
Qe          = zeros(3,length(t));
Qestimated  = zeros(nq,length(t));
Qmean       = zeros(nq,length(t));
u_mpc       = zeros(nu,length(t));
u_kc        = zeros(nu,length(t));
optiMPC.set_value(u_last,[0;0]);

% *************************************************************************
% Simulation
% *************************************************************************

% Initialization
optiMHE.set_value(uPast, repmat([0; 0], 1, Nh));  % Initialize past control inputs


optiMHE.set_value(xBar,gps0Bar_mean(:)); % Set initial state by the mean of the GPS measurements
optiMHE.set_value(y_meas,repmat(gps0Bar(:),1,Nh+1));
optiMHE.set_value(uPast,repmat([0;0],1,Nh));
solutionMHE = optiMHE.solve();



% *************************************************
% LPF Initialization
% ***********************************************
LPF_size = 8; % Size of the LPF
LPF_buff = repmat(gps0Bar_mean, 1, LPF_size); % Buffer for the LPF
Qlpf_estimated = zeros(nq, length(t)); % Estimated states by LPF



% Simulation loop

elapsed_time = zeros(1,2);

% Inicializar la figura y las leyendas
figure(1);
hold on;
grid on;
title('Trajectory Tracking');
xlim([-x_lim*1.1 x_lim*1.1])
ylim([-y_lim*1.1 y_lim*1.1])
xlabel('X');
ylabel('Y');
h1 = plot(NaN, NaN, 'k', 'DisplayName', 'Reference Trajectory', 'LineWidth', 1.5);
h2 = plot(NaN, NaN, 'b', 'DisplayName', 'Actual Position', 'LineWidth', 1.5);
h3 = plot(NaN, NaN, 'r--', 'DisplayName', 'Estimated Position', 'LineWidth', 1.5);
h4 = plot(NaN, NaN, 'g.', 'DisplayName', 'Mean Gps', 'LineWidth',1.5); 

legend([h1, h2, h3, h4]);

for i = 1:length(qr)
    %measure time of execution
    tic
    % MHE 
    % Update MHE
    qTrajEstimated  = solutionMHE.value(qMHE); % Get estimated state
    yTrajEstimated  = solutionMHE.value(y_meas); % Get estimated measurements
    uTrajEstimated  = solutionMHE.value(uPast); % Get estimated control inputs

    optiMHE.set_value(xBar,qTrajEstimated(:,2)); % Set initial state


    %GPS measurements
    last_gps = repmat(Q(1:2,i), 1, 4) + [cos(Q(3,i)), -sin(Q(3,i)); sin(Q(3,i)), cos(Q(3,i))]*0.5*[lg lg -lg -lg ; -wg wg wg -wg]; % GPS measurements
    last_measurement = [last_gps(:) + gps_noise_factor.*randn(size(last_gps(:)));Q(3,i)+gyro_noise_factor.*randn(1)]; % GPS and gyro measurements + noise
    Qmean(:,i) = [mean(last_measurement(1:2:end-1)); mean(last_measurement(2:2:end-1)); last_measurement(end)]; % Mean of the GPS measurements

    %MHE update
    optiMHE.set_value(y_meas,[yTrajEstimated(:,2:end),last_measurement]); % Set measurement
    optiMHE.set_value(uPast,[uTrajEstimated(:,2:end),u_mpc(:,i)]); % Set past control inputs
    solutionMHE = optiMHE.solve();  % Solve MHE
    optiMHE.set_initial(solutionMHE.value_variables); % Update initial condition
    Qaux            = solutionMHE.value(qMHE); % Get estimated state
    Qestimated(:,i) = Qaux(:,end); % Get estimated state

    %lpf update
    LPF_buff = [LPF_buff(:,2:end), Qmean(:,i)];
    Qlpf_estimated(:,i) = mean(LPF_buff, 2);



    if i < length(qr)
        % MPC
        optiMPC.set_value(qref, qr(:, i));  % Set reference
        optiMPC.set_value(qinit, Qestimated(:, i));  % Set initial state
        try
            solutionMPC = optiMPC.solve();  % Solve MPC
            optiMPC.set_initial(solutionMPC.value_variables());  % Update initial condition
            u_mpc(:, i+1) = solutionMPC.value(u(:, 1));  % Get control input
            optiMPC.set_value(u_last, u_mpc(:, i+1));  % Update last control input
        catch
            disp('MPC Solver failed');
            u_mpc(:, i+1) = u_mpc(:, i);  % Use previous control input
        end
        
        %measure time of execution
        elapsed_time(i) = toc;

        % Evolve system
        
        Q(:, i+1) = F(Q(:, i), u_mpc(:, i+1), Ts, w, zeros(3,1)); %+ [1.*randn(2,1);0.1.*randn(1)]; % Evolve system
    end

    %measure time of execution
    disp('Max elapsed time in seconds');
    disp(max(elapsed_time(:)));
    
    % Limpiar el gráfico y volver a dibujar las líneas
    cla;
    plot(qr(1,1:i), qr(2,1:i), 'k', 'DisplayName', 'Reference Trajectory', 'LineWidth',1.5); % Trajectory reference
    plot(Q(1,1:i), Q(2,1:i), 'b', 'DisplayName', 'Actual Position', 'LineWidth',1.5);   % Actual Position
    plot(Qestimated(1,1:i), Qestimated(2,1:i), 'r--', 'DisplayName', 'MHE Position', 'LineWidth',1.5); % Estimated Position
    plot(Qlpf_estimated(1,1:i), Qlpf_estimated(2,1:i), 'g.', 'DisplayName', 'Mean LPF Position', 'LineWidth',1.5); % Estimated Position
    %plot gps position
    %plot(last_gps(1,:),last_gps(2,:),'ko', 'DisplayName', 'GPS position');
    %plot gps measurements
    plot(last_measurement(1:2:end-1),last_measurement(2:2:end-1),'ro', 'DisplayName', 'GPS noisy measurements');
    drawnow;
end

% ******************************************************************
% Plot results
% ******************************************************************

figure(2); hold on; grid on;
plot(t,u_mpc(1,:), 'Color',[0.3010, 0.7450, 0.9330], 'DisplayName', 'MPC Control Input 1 VR'); 
plot(t,u_mpc(2,:), 'Color', [0.8500, 0.3250, 0.0980], 'DisplayName', 'MPC Control Input 2 VL');
legend('show')
title('Control Inputs');
xlabel('Time Step');
ylabel('Control Input');

figure(3); 

% Primer gráfico a la izquierda
subplot(1, 2, 1);
hold on; grid on;
plot(t, sqrt((Qestimated(1,:)-Q(1,:)).^2 + (Qestimated(2,:)-Q(2,:)).^2), 'r', 'DisplayName', 'Norm position error');
plot(t, sqrt((Qlpf_estimated(1,:)-Q(1,:)).^2 + (Qlpf_estimated(2,:)-Q(2,:)).^2), 'g', 'DisplayName', 'Norm position error Mean LPF'); 
legend('show');
title('Norm Position Error');
xlabel('Time Step');
ylabel('Norm Position Error');

% Segundo gráfico a la derecha
subplot(1, 2, 2); 
hold on; grid on;
plot(t, abs(Qestimated(3,:) - Q(3,:)), 'r', 'DisplayName', 'Error dir MHE');
plot(t, abs(Qlpf_estimated(3,:) - Q(3,:)), 'g', 'DisplayName', 'Error dir Mean LPF');
legend('show');
title('Error in Direction');
xlabel('Time Step');
ylabel('Error Direction');

%plot time of execution
figure(4); hold on; grid on;
plot( elapsed_time, 'b', 'DisplayName', 'Elapsed time by iteration');
legend('show');
title('Elapsed time by iteration');
xlabel('Iteration');
ylabel('Elapsed time in seconds');


%display rms error
disp('RMS error in position vs mean')
disp(rms(sqrt((Qlpf_estimated(1,:)-Q(1,:)).^2 + (Qlpf_estimated(2,:)-Q(2,:)).^2)) - rms(sqrt((Qestimated(1,:)-Q(1,:)).^2 + (Qestimated(2,:)-Q(2,:)).^2)));

disp('RMS error in direction vs mean')
disp(rms(abs(Qlpf_estimated(3,:) - Q(3,:))) - rms(abs(Qestimated(3,:) - Q(3,:)))); 

disp('Max error in position estimated perccentage compared to noise')
disp(max(sqrt((Qestimated(1,:)-Q(1,:)).^2 + (Qestimated(2,:)-Q(2,:)).^2)) / gps_noise_factor);


% Dynamics function
function Fk = F(q, u, Ts, w, noise)

    Fk = q + Ts * [cos(q(3)), 0; 
                   sin(q(3)), 0; 
                   0, 1] *[1/2 1/2; 1/w -1/w]* u + noise;
end


