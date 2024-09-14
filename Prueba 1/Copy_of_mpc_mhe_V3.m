clear all
close all
clc

addpath('C:\Users\nicolas\Desktop\Memoria\casadi-3.6.5-windows64-matlab2018b')
import casadi.*

% *************************************************************************
% general Parameters:
% *************************************************************************
Ts      = 0.1;                  % Sampling time

% *************************************************************************
% Vehicle's geometrical properties:
% *************************************************************************
l = 0.2;  % This parameter is kept because it is used in the dynamics function

% *************************************************************************
% System's properties:
% *************************************************************************
nq      = 3;                        % number of states (x, y, theta)
nu      = 2;                        % number of inputs (controls (vr , vl))
nr      = 3;                        % number of states to be controlled

% *************************************************************************
% Model predictive parameters:
% *************************************************************************
Nc      = 10;                       % MPC's control horizon length
dU_lb   = [-60; -30];                 % lower bound for the accelerations
dU_ub   = [60; 30];                   % upper bound for the accelerations

optiMPC = casadi.Opti();            % Define Casadi variable for building the MPC problem

% Definition of the MPC's variables and problem:
qMPC    = optiMPC.variable(nq, Nc+1);    % Predicted trajectories
u       = optiMPC.variable(nu, Nc);      % The controls
qref    = optiMPC.parameter(nr);         % The reference for the controller
qinit   = optiMPC.parameter(nq);         % Initial condition
u_last  = optiMPC.parameter(nu);         % Control applied in the last sampling time

% Matrices para los costos cuadráticos por etapa
Wmpc    = diag([0.1 1 1]);              % Matrices de peso
Rmpc    = diag([0.01 0.05]);            % Penalizar valores grandes de los controles

% *************************************************************************
% MPC Objective function
% *************************************************************************
Jmpc = 0;
for k=1:Nc+1
    Jmpc = Jmpc + (qMPC(:,k) - qref).' * Wmpc * (qMPC(:,k) - qref);
    if k < Nc+1
        optiMPC.subject_to(qMPC(:,k+1) == F(qMPC(:,k), u(:,k), Ts,l,zeros(3,1)));     % q_{k+1} = f(q_k, u_k)
        if k == 1
            %optiMPC.subject_to( Ts * dU_lb <= u(:,k) <= Ts * dU_ub );  % Constraints on the accelerations
        else
            %optiMPC.subject_to( Ts * dU_lb <= u(:,k) - u(:,k-1) <= Ts * dU_ub );% Constraints on the accelerations
        end
        Jmpc = Jmpc + u(:,k).' * Rmpc * u(:,k);                                 % Penalizar valores grandes de los controles
    end
end

optiMPC.minimize(Jmpc);                                                      % Minimizar la función objetivo
%optiMPC.subject_to([-10; -10] < u < [10; 10]);                   % Constraints on the control inputs
optiMPC.subject_to(qMPC(:,1) == qinit);                                      % Set the initial state

% Some parameters for the solver (do not care about that, for now...)
p_optsMPC = struct('expand',true);
s_optsMPC = struct('sb','yes','print_level',0,'gamma_theta',1e-2,'jacobian_approximation','exact','fast_step_computation','yes','warm_start_init_point','yes'); % 
optiMPC.solver('ipopt',p_optsMPC,s_optsMPC);   

% *************************************************************************
% MHE Parameters
% *************************************************************************
Nh = 10;                       % MHE horizon length
optiMHE = casadi.Opti();       % Define Casadi variable for building the MHE problem

qMHE = optiMHE.variable(nq, Nh+1);     % State estimates
w = optiMHE.variable(nq, Nh);          % Process noise
v = optiMHE.variable(nq, Nh);          % Measurement noise
y_meas = optiMHE.parameter(nq, Nh+1);  % Measurements
q0_mhe = optiMHE.parameter(nq);        % Initial state for MHE
uPast   = optiMHE.parameter(nu,Nh);    % Past control inputs applied to the system

% Matrices for the quadratic stage-costs in MHE
Wmhe = diag([0.1 1 1]);
Vmhe = diag([0.1 1 1]);

% *************************************************************************
% MHE Objective function
% *************************************************************************
Jmhe = 0;

for k = 1:Nh+1
    Jmhe = Jmhe + (y_meas(:,k) - qMHE(:,k)).' * Wmhe * (y_meas(:,k) - qMHE(:,k));
    if k < Nh+1
        optiMHE.subject_to(qMHE(:,k+1) == F(qMHE(:,k), uPast(:,k), Ts,l,w(:,k)) + w(:,k));  % State transition
        Jmhe = Jmhe + w(:,k).' * Vmhe * w(:,k);                                      % Process noise cost
    end
end

optiMHE.minimize(Jmhe);                                            % Minimize the MHE objective
optiMHE.subject_to(qMHE(:,1) == q0_mhe);                           % Initial state constraint
optiMHE.subject_to(-0.1 <= w <= 0.1);                              % Process noise bounds
optiMHE.subject_to(-0.1 <= v <= 0.1);                              % Measurement noise bounds
optiMHE.solver('ipopt');                                           % Solve the MHE problem

% Some parameters for the solver (do not care about that, for now...)
p_optsMPC = struct('expand',true);
s_optsMPC = struct('sb','yes','print_level',0,'gamma_theta',1e-2,'jacobian_approximation','exact','fast_step_computation','yes','warm_start_init_point','yes'); % 
optiMPC.solver('ipopt',p_optsMPC,s_optsMPC);   

% *************************************************************************
% Define path to follow
% *************************************************************************
t = 0:0.01:2*pi;
x_path = 10*cos(t);
y_path = 5.*sin(t);
thetaref = unwrap(atan2(diff(y_path), diff(x_path)));
qr = [x_path; y_path;[thetaref, thetaref(end)]];           % Reference trajectory
 
optiMPC.set_value(u_last, [0;0]);                                    % Initialize the last control input

q0 = [1; 0; pi/2];                                          % Initial state
Q = zeros(nq, length(t));
Q(:,1) = qr(:,1);
Qr = zeros(3, length(t));
u_mpc = zeros(nu, length(t));
Qmhe = zeros(nq, length(t));                                 % MHE estimates
Qmhe(:,1) = q0;
y_meas_data = Q + 0*0.01*randn(nq, length(t));                  % Noisy measurements

% *************************************************************************
% Simulation
% *************************************************************************

%************************************************
%Initialization%

optiMHE.set_value(uPast,repmat([0;0],1,Nh)); %establish all past u as 0

%*******************************************************************
for i = 1:length(qr) - 1
    % MHE
    if i+Nh <629
        optiMHE.set_value(y_meas, y_meas_data(:,i:i+Nh));   % Set measurements for MHE
    else
        break;   % Set measurements for MHE
    end
    optiMHE.set_value(q0_mhe, Qmhe(:,i));                        % Set the initial state for MHE
    solutionMHE = optiMHE.solve();                               % Solve the MHE problem
    Qmhe(:,i+1) = solutionMHE.value(qMHE(:,2));                  % Get the estimated state
    
    % % MPC
    % optiMPC.set_value(qref, qr(:,i));                            % Set the reference for the controller
    % optiMPC.set_value(qinit, Qmhe(:,i));                       % Set the initial state (from MHE)
    % solutionMPC = optiMPC.solve();                               % Solve the MPC problem
    % 
    % 
    % optiMPC.set_initial(solutionMPC.value_variables());          % Set initial condition for next solve
    % Qtraj = solutionMPC.value(qMPC);                             % Predicted trajectory
    % Utraj = solutionMPC.value(u);
    % u_mpc(:,i+1) = Utraj(:,1);
    % optiMPC.set_value(u_last, u_mpc(:,i+1));
    u_mpc(:,i+1)=1*[1 1 1; 1 1 1]*(Qmhe(:,i+1)-qr(:,i+1));
    
    % x=Qmhe(1,i+1)-qr(1,i+1)
    % y=Qmhe(2,i+1)-qr(2,i+1)
    % th=Qmhe(3,i+1)-qr(3,i+1)

    
    
    % Evolve the system
    Q(:,i+1) = F(Q(:,i), u_mpc(:,i+1), Ts,l,zeros(3,1));
end

% Plotting results
figure; hold on; grid on;
plot(qr(2,:), qr(3,:), 'k', 'DisplayName', 'Reference Trajectory')
plot(Q(2,:), Q(3,:), 'y', 'DisplayName', 'Actual Position')
plot(Qmhe(2,:), Qmhe(3,:), 'r--', 'DisplayName', 'Estimated Position')
legend('show')

figure; hold on; grid on;
plot(u_mpc(1,:), 'g', 'DisplayName', 'MPC Control Input 1'); 
plot(u_mpc(2,:), 'b', 'DisplayName', 'MPC Control Input 2')
legend('show')

% Dynamics function
function Fk = F(q, u, Ts,l,w)
    Fk = q + Ts * [cos(q(3)),0; 
                   sin(q(3)),0; 
                   0,1] *[1/2 , 1/2; 
                            1/l , -1/l]*[u(1); u(2)]+w;
end
