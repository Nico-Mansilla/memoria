%En este codigo se logra implementar mhe con mpc para el seguimiento de un
%vehiculo simple y sin ruido de una referencia obalada
%
%
%

clear all
close all
clc

addpath('C:\Users\nicolas\Desktop\Memoria\casadi-3.6.5-windows64-matlab2018b')
import casadi.*

% *************************************************************************
% General Parameters:
% *************************************************************************
Ts = 0.1;  % Sampling time

% *************************************************************************
% Vehicle's Geometrical Properties:
% *************************************************************************
w = 0.2;  % Robot width (between wheels)
l = 0.4;  % Robot length 

wg = 0.2;  % GPS width 
lg = 0.4;  % GPS length 

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
dU_lb = [-10; -10];  % Lower bound for the control inputs
dU_ub = [10; 10];    % Upper bound for the control inputs

% Create Casadi optimization object
optiMPC = casadi.Opti();

% Define MPC variables
qMPC = optiMPC.variable(nq, Nc+1);  % Predicted state trajectories
u = optiMPC.variable(nu, Nc);       % Control inputs
qref = optiMPC.parameter(nr);       % Reference state
qinit = optiMPC.parameter(nq);      % Initial state
u_last = optiMPC.parameter(nu);     % Last control input

% Cost matrices
Wmpc = diag([1 1 1]);    % State cost weight matrix
Rmpc = diag([1*0.001 1*0.001]);  % Control input cost weight matrix

% *************************************************************************
% MPC Objective Function
% *************************************************************************
Jmpc = 0;
for k = 1:Nc+1
    Jmpc = Jmpc + (qMPC(:,k) - qref).' * Wmpc * (qMPC(:,k) - qref);
    if k < Nc+1
        optiMPC.subject_to(qMPC(:,k+1) == F(qMPC(:,k), u(:,k), Ts, w, zeros(3,1)));
        if k == 1
            optiMPC.subject_to((dU_lb <= u(:,k)) <= dU_ub);
        else
            optiMPC.subject_to((dU_lb <= u(:,k) - u(:,k-1)) <= dU_ub);
        end
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
Nh = 10;  % MHE horizon length
optiMHE = casadi.Opti();

qMHE = optiMHE.variable(nq, Nh+1);  % State estimates
wMHE = optiMHE.variable(nq, Nh);       % Process noise
vMHE = optiMHE.variable(nq, Nh+1);       % Measurement noise
y_meas = optiMHE.parameter(nq, Nh+1);  % Measurements

uPast = optiMHE.parameter(nu, Nh);     % Past control inputs applied to the system
X       = optiMHE.variable(nq);         % Arrival-cost
xBar    = optiMHE.parameter(nq);        % Initial state for MHE

% Cost matrices for MHE
Rmhe = diag([1 1 1]); %q pert
Qmhe = diag([1 1 1]); % measure pert
Pmhe = diag([1 1 1]); %arribal cost

% *************************************************************************
% MHE Objective Function
% *************************************************************************
% Set initial state constraint
%optiMHE.subject_to(qMHE(:,1) == xBar);

Jmhe = X.'*Pmhe*X; % ARRIVAL-COST
optiMHE.subject_to(X == qMHE(:,1)-xBar);

for k = 1:Nh
    Jmhe = Jmhe + vMHE(:,k).'*Rmhe*vMHE(:,k) + wMHE(:,k).'*Qmhe*wMHE(:,k);
    optiMHE.subject_to(qMHE(:,k+1) == F(qMHE(:,k), uPast(:,k), Ts, w, wMHE(:,k)));
    optiMHE.subject_to( y_meas(:,k) == qMHE(:,k) + vMHE(:,k) ); 
end
optiMHE.subject_to( y_meas(:,Nh+1) == qMHE(:,Nh+1) + vMHE(:,Nh+1) ); 
Jmhe = Jmhe + vMHE(:,Nh+1).'*Rmhe*vMHE(:,Nh+1);%+ wMHE(:,Nh+1).'*Qmhe*wMHE(:,Nh+1);

% Set objective
optiMHE.minimize(Jmhe);



% Bounds on process and measurement noise
optiMHE.subject_to((-0.1 <= wMHE) <= 0.1);
optiMHE.subject_to((-0.1 <= vMHE) <= 0.1);

% *************************************************************************

% Some parameters for the solver (do not care about that, for now...)
p_optsMHE = struct('expand',true);
s_optsMHE = struct('sb','yes','print_level',0,'gamma_theta',1e-2,'jacobian_approximation','exact','fast_step_computation','yes','warm_start_init_point','yes'); % 
optiMHE.solver('ipopt',p_optsMHE,s_optsMHE);


% *************************************************************************
% Define Path to Follow
% *************************************************************************
t           = 0:0.02:2*pi;
x_lim       = 20;
y_lim       = 10;
x_path      = x_lim*cos(t);
y_path      = y_lim*sin(t);
thetaref    = unwrap(atan2(diff(y_path),diff(x_path)));
qr          = [ x_path; y_path;[thetaref,thetaref(end)]];           % First reference. This reference need to be updated then in every sampling time
% *************************************************************************

optiMPC.set_value(u_last,[0;0]);                                % init this parameter of the MPC
%

q0          = [0,0,0];    % This function generates a feasible initila condition: The vehicle is a challenging one, 
q0Bar       = q0 + 0.01.*randn(size(q0));  %first measure 
Q           = zeros(nq,length(t));
Q(:,1)      = q0;
Qr          = zeros(3,length(t));
Qe          = zeros(3,length(t));
Qestimated  = zeros(nq,length(t));
u_mpc       = zeros(nu,length(t));
u_kc        = zeros(nu,length(t));
optiMPC.set_value(u_last,[0;0]);

% *************************************************************************
% Simulation
% *************************************************************************

% Initialization
optiMHE.set_value(uPast, repmat([0; 0], 1, Nh));  % Initialize past control inputs
%optiMHE.set_value(xBar,q0Bar);

optiMHE.set_value(xBar,q0Bar);
optiMHE.set_value(y_meas,repmat(q0Bar(:),1,Nh+1));
optiMHE.set_value(uPast,repmat([0;0],1,Nh));
solutionMHE = optiMHE.solve();


% Simulation loop

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
%h4 = plot(NaN, NaN, 'm--', 'DisplayName', 'Mean Position', 'LineWidth',1.5); 

%legend([h1, h2, h3, h4]);
legend([h1, h2, h3]);

for i = 1:length(qr)
    % MHE
    qTrajEstimated  = solutionMHE.value(qMHE);
    yTrajEstimated  = solutionMHE.value(y_meas);
    uTrajEstimated  = solutionMHE.value(uPast);

    optiMHE.set_value(xBar,qTrajEstimated(:,2));
    last_measurement = Q(:,i); % + 0.1.*randn(nq,1); % Simulación de ruido
    optiMHE.set_value(y_meas,[yTrajEstimated(:,2:end),last_measurement]);
    optiMHE.set_value(uPast,[uTrajEstimated(:,2:end),u_mpc(:,i)]);
    solutionMHE = optiMHE.solve();
    optiMHE.set_initial(solutionMHE.value_variables);
    Qaux            = solutionMHE.value(qMHE);
    Qestimated(:,i) = Qaux(:,end);

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
    
        % Evolve system
        Q(:, i+1) = F(Q(:, i), u_mpc(:, i+1), Ts, w, zeros(3,1));
    end
    
    % Limpiar el gráfico y volver a dibujar las líneas
    cla;
    plot(qr(1,1:i), qr(2,1:i), 'k', 'DisplayName', 'Reference Trajectory', 'LineWidth',1.5); % Trajectory reference
    plot(Q(1,1:i), Q(2,1:i), 'b', 'DisplayName', 'Actual Position', 'LineWidth',1.5);   % Actual Position
    plot(Qestimated(1,1:i), Qestimated(2,1:i), 'r--', 'DisplayName', 'Estimated Position', 'LineWidth',1.5); % Estimated Position
    %plot(Qmean(1,1:i), Qmean(2,1:i), 'm--', 'DisplayName', 'Mean Position', 'LineWidth',1.5); % Estimated Position
    drawnow;
end


figure(2); hold on; grid on;
plot(t,u_mpc(1,:), 'g', 'DisplayName', 'MPC Control Input 1 VR', 'LineWidth',1.5); 
plot(t,u_mpc(2,:), 'b', 'DisplayName', 'MPC Control Input 2 VL', 'LineWidth',1.5)
legend('show')
title('Control Inputs');
xlabel('Time Step');
ylabel('Control Input');

figure(3); 

% Primer gráfico a la izquierda
subplot(1, 2, 1); % 1 fila, 2 columnas, primer gráfico
hold on; grid on;
plot(t, sqrt((Qestimated(1,:)-Q(1,:)).^2 + (Qestimated(2,:)-Q(2,:)).^2), 'g', 'DisplayName', 'Norm position error', 'LineWidth',1.5); 
legend('show');
title('Norm Position Error');
xlabel('Time Step');
ylabel('Norm Position Error');

% Segundo gráfico a la derecha
subplot(1, 2, 2); % 1 fila, 2 columnas, segundo gráfico
hold on; grid on;
plot(t, Qestimated(3,:) - Q(3,:), 'b', 'DisplayName', 'Error dir', 'LineWidth',1.5);
legend('show');
title('Error in Direction');
xlabel('Time Step');
ylabel('Error Direction');
% Dynamics function
function Fk = F(q, u, Ts, w, noise)
    Fk = q + Ts * [cos(q(3)), 0; 
                   sin(q(3)), 0; 
                   0, 1] *[1/2 1/2; 1/w -1/w]* u + noise;
end


