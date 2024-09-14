clear all
close all
clc

import casadi.*
% *************************************************************************
% general Parameters:
% *************************************************************************
Ts      = 0.1;                  % Sampling time

% *************************************************************************
% Vehicle's (Tractor pulling 2 trailers) geometrical proeprties:
% *************************************************************************
Nt      = 2;                            % Number of passive trailers
Lh1     = 0.342;                        % geometrical properties of the vehicle
L1      = 1.08;
Lh2     = 0;
L2      = 0.78;

% *************************************************************************
% System's properties:
% *************************************************************************
nq      = 4*Nt+3;                       % number of states
nu      = 2;                            % number of inputs (controls)
nr      = 3;                            % number of staes to be controlled or steered
indxOutput = [1:Nt+1,2*Nt+2,2*Nt+3];
ny      = Nt+3;                         % number of states measured (for the MHE): h(q_t)=(b_1,..,b_Nt,theta_0,x0,y0). Note that the attitude and position of the trailers are not measured!!!

% *************************************************************************
% Model predictive parameters:
% *************************************************************************
Nc      = 15;                           % MPC's control horizon length
dU_lb   = [-6; -3];                     % lower blund for the accelerations
dU_ub   = [6; 3];                       % upper bound for the accelerations

optiMPC = casadi.Opti();                % Definde Casadi variable for building the MPC problem


% Definition of the MPC's varibales and problem:
qMPC    = optiMPC.variable(nq,Nc+1);    % This variables a re optimisation variables: the predictes trajectories
u       = optiMPC.variable(nu,Nc);      % The controls: What we are goin to apply to the vehicle  
qref    = optiMPC.parameter(nr);        % The reference for the controller: wehre we want to steer the vehicle
qinit   = optiMPC.parameter(nq);        % intial condition
u_last  = optiMPC.parameter(nu);        % Control applied in the las sampling time: used for constraint the acceleration of the first control in the sequnece

% matrices for the quadratic stage-costs
Wmpc    = diag([0.1 1 1]);              % Weight matrices: penalises deviation with respect to the reference
Rmpc    = diag([0.01 0.05]);            % Penalise lqrge values of the controls

% *************************************************************************
% MPC Objective function
% *************************************************************************
Jmpc = 0;
for k=1:Nc+1
    Jmpc = Jmpc + (qMPC([Nt+1,2*Nt+2:2*Nt+3],k)-qref).'* Wmpc *(qMPC([Nt+1,2*Nt+2:2*Nt+3],k)-qref);
    if k < Nc+1
        optiMPC.subject_to(qMPC(:,k+1) == F(qMPC(:,k),u(:,k),Nt,Ts,Lh1,L1,Lh2,L2));     % This is: q_{k+1}=f(q_k,u_k)
        if k==1
            optiMPC.subject_to( Ts.*dU_lb <= u(:,k)-u_last <= Ts.*dU_ub );              % Constraints on the accelerations
        else
            optiMPC.subject_to( Ts.*dU_lb <= u(:,k)-u(:,k-1) <= Ts.*dU_ub );            % Constraints on the accelerations
        end        
        %
        Jmpc = Jmpc + u(:,k).'* Rmpc *u(:,k);                                           % Penalise large values of the controls
    end
end

optiMPC.minimize(Jmpc);                                                     % What we are goin to minimise
optiMPC.subject_to([-45*pi/180;-1] < u < [45*pi/180;1]);                    % restriccion velocidades angular y lineal
optiMPC.subject_to(qMPC(:,1) == qinit);                                     % Set the first state as the inital state, which will be computed latyer by the MHE
% Some parameters for the solver (do not care about that, for now...)
p_optsMPC = struct('expand',true);
s_optsMPC = struct('sb','yes','print_level',0,'gamma_theta',1e-2,'jacobian_approximation','exact','fast_step_computation','yes','warm_start_init_point','yes'); % 
optiMPC.solver('ipopt',p_optsMPC,s_optsMPC);                                % MPC probelm is solved

% *************************************************************************
% Now, let's define the MHE
% *************************************************************************
Ne      = 5;                            % MHE's window's length

optiMHE = casadi.Opti();                % casadi's optimisation problem variable
qMHE    = optiMHE.variable(nq,Ne+1);    % Trajectory (of states) to be minimised
yMHE    = optiMHE.parameter(ny,Ne+1);   % Measurements of the output of the system
wMHE    = optiMHE.variable(nq,Ne);      % Estimation of the process disturbance variables
vMHE    = optiMHE.variable(ny,Ne+1);    % Residuals of the estimation
uPast   = optiMHE.parameter(nu,Ne);     % Past control inputs applied to the system
X       = optiMHE.variable(nq);         % Arrival-cost
xBar    = optiMHE.parameter(nq);        % Arrival-cost
Pmhe    = eye(nq);                      % Matrix for the arrival-cost
Wmhe    = eye(nq);                      % Matrix for weighin the process disturbance
Rmhe    = eye(ny);                      % Matrix for weighin the residuals of the estimation
% *************************************************************************
% This is the Objective function for the MHE problem:
% *************************************************************************
Jmhe = X.'*Pmhe*X; % ARRIVAL-COST
optiMHE.subject_to(X == qMHE(:,1)-xBar);
for i=1:Ne
    Jmhe = Jmhe + (wMHE(:,i).'*Wmhe*wMHE(:,i)) + (vMHE(:,i).'*Rmhe*vMHE(:,i));
    optiMHE.subject_to( qMHE(:,i+1) == F(qMHE(:,i),uPast(:,i),Nt,Ts,Lh1,L1,Lh2,L2) + wMHE(:,i) );
    for j=1:Nt
        optiMHE.subject_to( qMHE(j,i) == qMHE(Nt+j,i)-qMHE(Nt+1+j,i) );
    end
    optiMHE.subject_to( yMHE(:,i) == qMHE(indxOutput,i) + vMHE(:,i) ); 
end
optiMHE.subject_to( yMHE(:,Ne+1) == qMHE(indxOutput,Ne+1) + vMHE(:,Ne+1) );
for j=1:Nt
    optiMHE.subject_to( qMHE(j,Ne+1) == qMHE(Nt+j,Ne+1)-qMHE(Nt+1+j,Ne+1) );
end
Jmhe = Jmhe + vMHE(:,Ne+1).'*Rmhe*vMHE(:,Ne+1);
% *************************************************************************
optiMHE.minimize(Jmhe);                 % Minimise the OF
% Some parameters for the solver (do not care about that, for now...)
p_optsMHE = struct('expand',true);
s_optsMHE = struct('sb','yes','print_level',0,'gamma_theta',1e-2,'jacobian_approximation','exact','fast_step_computation','yes','warm_start_init_point','yes'); % 
optiMHE.solver('ipopt',p_optsMHE,s_optsMHE);

% *************************************************************************
% Define path to follow
% *************************************************************************
t           = 0:0.01:2*pi;
x_path      = 10*cos(t);
y_path      = 5.*sin(t);
thetaref    = unwrap(atan2(diff(y_path),diff(x_path)));
qr          = [[thetaref,thetaref(end)]; x_path; y_path];           % First reference. This reference need to be updated then in every sampling time
% *************************************************************************

optiMPC.set_value(u_last,[0;0]);                                % init this parameter of the MPC
%

q0          = gen_x0(Nt,[0;0;pi/2],qr(2:3,1),Lh1,L1,Lh2,L2);    % This function generates a feasible initila condition: The vehicle is a challenging one, 
q0Bar       = q0 + 0.1.*randn(size(q0));                        % It simulates the initial uncertainiuy about the initial condition of the system, i.e., in real life, I do no knwon exactly the position, for example. Though I can,measure it, this measuremnt is affected by noise
Q           = zeros(nq,length(t));
Q(:,1)      = q0;
Qr          = zeros(3,length(t));
Qe          = zeros(3,length(t));
Qestimated  = zeros(nq,length(t));
u_mpc       = zeros(nu,length(t));
u_kc        = zeros(nu,length(t));
optiMPC.set_value(u_last,[0;0]);
timesMPC    = zeros(1,length(t));
timesMHE    = zeros(1,length(t));
% *************************************************************************
% Init MHE: 
% *************************************************************************
optiMHE.set_value(xBar,q0Bar);
optiMHE.set_value(yMHE,repmat(q0Bar(indxOutput),1,Ne+1));
optiMHE.set_value(uPast,repmat([0;0],1,Ne));
solutionMHE = optiMHE.solve();

% *************************************************************************
% Finally, the simulation
% *************************************************************************
for i=1:length(qr)-1
    % *********************************************************************
    % First solve the MHE problem
    % *********************************************************************
    tic;
    qTrajEstimated  = solutionMHE.value(qMHE);
    yTrajEstimated  = solutionMHE.value(yMHE);
    uTrajEstimated  = solutionMHE.value(uPast);
    optiMHE.set_value(xBar,qTrajEstimated(:,2));
    last_measurement = Q(indxOutput,i);% + 0.1.*randn(ny,1);       % QUE PASA SIA GREGAMOS RUIDO A LAS MEDICIONES???!!!! SENSORES REALES NO SON PERFECTOS, Y POR MAS BUENOS QUE SEAN, SIEMPRE TIENEN ALGO DE RUIDO
    optiMHE.set_value(yMHE,[yTrajEstimated(:,2:end),last_measurement]);
    optiMHE.set_value(uPast,[uTrajEstimated(:,2:end),u_mpc(:,i)]);
    solutionMHE = optiMHE.solve();
    optiMHE.set_initial(solutionMHE.value_variables);
    Qaux            = solutionMHE.value(qMHE);
    Qestimated(:,i) = Qaux(:,end);
    timesMHE(i)     = toc;
    % *********************************************************************
    % Once I solved the MHE, I feed the system state to the MPC
    tic;
    % *********************************************************************
    % Now I can solve the MPC problem
    % *********************************************************************
    optiMPC.set_value(qref,qr(:,i));                        % Set the reference for the controller
    optiMPC.set_value(qinit,Qestimated(:,i));                        % Set the initial condition for the controller, computed with the MHE
    solutionMPC     = optiMPC.solve();                      % Solve the MPC problem
    optiMPC.set_initial(solutionMPC.value_variables());     % This is just for the internal solver: set an numerial initial condition for next time to be solved
    Qtraj           = solutionMPC.value(qMPC);              % Predicted trajectory
    Utraj           = solutionMPC.value(u);
    u_mpc(:,i+1)    = Utraj(:,1);
    optiMPC.set_value(u_last,u_mpc(:,i+1));
    timesMPC(i)     = toc;
    % *********************************************************************
    % Evolve the system: here the 'real system' is simulated
    % *********************************************************************
    Q(:,i+1) = F(Q(:,i),u_mpc(:,i+1),Nt,Ts,Lh1,L1,Lh2,L2);
    % *********************************************************************    
    % Do some nice pltos ;-)
    plot_mono2(Qestimated(:,i),Qtraj,x_path,y_path,qr(2*Nt+2),qr(2*Nt+3),L1,L2);
end
% *************************************************************************

figure; hold on; grid on;
plot(qr(2,:),qr(3,:))
plot(Qestimated(2*Nt+2,1:end-1),Qestimated(2*Nt+3,1:end-1),'g-.',Qestimated(2*Nt+4,1:end-1),Qestimated(2*Nt+5,1:end-1),'c-.',Qestimated(2*Nt+6,1:end-1),Q(2*Nt+7,1:end-1),'m-.')
plot(Q(2*Nt+2,:),Q(2*Nt+3,:),'y',Q(2*Nt+4,:),Q(2*Nt+5,:),'b',Q(2*Nt+6,:),Q(2*Nt+7,:),'r')
daspect([1 1 1])

figure; hold on; grid on;
plot(u_mpc(1,:),'g'); plot(u_mpc(2,:),'b')
plot(u_kc(1,:),'g-.'); plot(u_kc(2,:),'b-.')






function Fk = F(q,u,Nt,Ts,Lh1,L1,Lh2,L2)
    J1 = [-Lh1 * cos(q(1)) / L1, sin(q(1))/L1; Lh1*sin(q(1)), cos(q(1))];
    J2 = [-Lh2 * cos(q(2)) / L2, sin(q(2))/L2; Lh2*sin(q(2)), cos(q(2))];      
    
    f_rhs  = [  [1 0] * (eye(2)-J1) * u;
                [1 0] * (eye(2)-J2) * J1 * u;
                %
                [1 0] * u;
                [1 0] * J1 * u;
                [1 0] * J2 * J1 * u;
                %
                [0 cos(q(Nt+1))] * u;
                [0 sin(q(Nt+1))] * u;
                [0 cos(q(Nt+2))] * J1 * u;
                [0 sin(q(Nt+2))] * J1 * u;
                [0 cos(q(Nt+3))] * J2 * J1 * u;
                [0 sin(q(Nt+3))] * J2 * J1 * u];
    
    
    Fk = q + Ts.*f_rhs;
end

function x0 = gen_x0(Nt,x_y_theta,x_y_ref,Lh1,L1,Lh2,L2)
    % DO NOT FORGET CHECKING FEASIBILITY OF INITIAL CONDITION!!!
    if nargin == 1
        Dx          = x_y_ref(1,2)-x_y_ref(1,1);
        Dy          = x_y_ref(2,2)-x_y_ref(2,1);
        theta0      = atan2(Dy,Dx);
        thetas      = repmat(theta0,Nt+1,1);
        xy_0        = x_y_ref(:,1);
    else        
        thetas      = [x_y_theta(3:end)'; repmat(x_y_theta(end),Nt+1-length(x_y_theta(3:end)),1)];
        xy_0        = x_y_theta(1:2);%[xAux;yAux];
    end
    betas   = -diff(thetas);    
    %
    xy_1    = xy_0 - [Lh1*cos(thetas(1))+L1*cos(thetas(2)); Lh1*sin(thetas(1))+L1*sin(thetas(2))];
    xy_N    = xy_1 - [Lh2*cos(thetas(2))+L2*cos(thetas(3)); Lh2*sin(thetas(2))+L2*sin(thetas(3))];
    xy_0toN = [xy_0;xy_1;xy_N];
    x0      = [ betas; thetas; xy_0toN];
end

function plot_mono2(qk,Qtraj,xpath,ypath,xref,yref,L1,L2)
    clrWheel            = 'k';
    clrAxe              = 'k';
    clrLongAxe          = 'k';
    clrTrailerLoad      = 'b';
    clrLastTrailerLoad  = 'r';
    noRef               = false;
   
    coordinates         = [xpath;ypath];
    Nt                  = 2;
    segmentTosteer      = 0;
    ref                 = [xref;yref];
    xReachable          = xref;
    yReachable          = yref;
    XYtrailerAxe        = [-0.67/2 0.67/2; 0 0];
    XYtrailerWheelLeft  = [-0.67/2 -0.67/2; 0.15/2 -0.15/2];
    XYtrailerWheelRight = [0.67/2 0.67/2; -0.6/2 0.15/2];
    XYtrailerLongAxe    = [[0 0; 0 L1];[0 0; 0 L2]];
    XYtracWheelLeft     = [-0.67/2 -0.67/2; 0.15/2 -0.15/2];
    XYtracWheelRight    = [0.67/2 0.67/2; -0.15/2 0.15/2];
    XYtracAxe           = [-0.67/2 0.67/2; 0 0];

    if ~noRef        
        % Plot the path
        plot(coordinates(1,:),coordinates(2,:),'color',[0.7 0.7 0.7],'LineWidth',8); grid on; daspect([1 1 1]); hold on;
        % Plot the predicted trajectory for each segment of the N-trailer
        for i=1:Nt+1
            if i==1
                plot(Qtraj(2*Nt+1+(i-1)*2+1,:),Qtraj(2*Nt+1+i*2,:),'y','LineWidth',4);
            elseif i~=Nt+1
                plot(Qtraj(2*Nt+1+(i-1)*2+1,:),Qtraj(2*Nt+1+i*2,:),'b','LineWidth',4);
            else
                plot(Qtraj(2*Nt+1+(i-1)*2+1,:),Qtraj(2*Nt+1+i*2,:),'r','LineWidth',4);
            end
        end
    end    
    % Plot the current reference
    if segmentTosteer == 0
        clr         = 'y';
        clrReach    = [0.95 0.95 0.1];
    elseif segmentTosteer == Nt
        clr         = 'r';
        clrReach    = [1 0.1 0.1];
    else
        clr = 'b';
        clrReach    = [0.1 0.1 1];
    end
    plot(xref,yref,clr,'marker','+','linewidth',2,'markersize',15)
    plot(xref,yref,clr,'marker','o','linewidth',2,'markersize',15)
    % plot the reachable reference
    plot(xReachable,yReachable,'color',clrReach,'marker','+','linewidth',2,'markersize',15)
    plot(xReachable,yReachable,'color',clrReach,'marker','o','linewidth',2,'markersize',15)
    %
    thetas  = qk(Nt+1:2*Nt+1); 
    xy0     = qk(2*Nt+2:2*Nt+3);
    xyi     = zeros(2,Nt+1);
    xyi(:,1) = xy0;
    %
    R                       = [cos(thetas(1)-pi/2), -sin(thetas(1)-pi/2); sin(thetas(1)-pi/2), cos(thetas(1)-pi/2)];
    Rh                      = [cos(-thetas(1)-pi/2), -sin(-thetas(1)-pi/2); sin(-thetas(1)-pi/2), cos(-thetas(1)-pi/2)];

%     tractorBodyPlt          = S.system.XYtracBody*Rh + xyi(:,1);
%     hold on;
%     tractorBodyPlt.plot('color','y');
%     hold off;


    tractorWheelLeftPlt     = R * XYtracWheelLeft + xyi(:,1);
    tractorWheelRightPlt    = R * XYtracWheelRight + xyi(:,1);
    tractorAxePlt           = R * XYtracAxe + xyi(:,1);

    line(tractorWheelLeftPlt(1,:),tractorWheelLeftPlt(2,:),'color',clrWheel,'linewidth',4);
    line(tractorWheelRightPlt(1,:),tractorWheelRightPlt(2,:),'color',clrWheel,'linewidth',4);
    line(tractorAxePlt(1,:),tractorAxePlt(2,:),'color',clrAxe,'linewidth',2);
    %
    for i=1:Nt
        R                       = [cos(thetas(i+1)-pi/2), -sin(thetas(i+1)-pi/2); sin(thetas(i+1)-pi/2), cos(thetas(i+1)-pi/2)];
        Rtr                     = [cos(-thetas(i+1)+pi/2), -sin(-thetas(i+1)+pi/2); sin(-thetas(i+1)+pi/2), cos(-thetas(i+1)+pi/2)];
        trailerAxePlt           = R * XYtrailerAxe + xyi(:,i+1);
        trailerWheelLeftPlt     = R * XYtrailerWheelLeft + xyi(:,i+1);
        trailerWheelRightPlt    = R * XYtrailerWheelRight + xyi(:,i+1);
        trailerLongAxePlt       = R * XYtrailerLongAxe((i-1)*2+1:i*2,:) + xyi(:,i+1);
%         trailerLoadPlt          = XYtrailerLoad{i}*Rtr + xyi(:,i+1);
        hold on;
%         if i==Nt
%             trailerLoadPlt.plot('color','r');
%         else
%             trailerLoadPlt.plot('color','b');
%         end

        if i~= Nt
            trailerClr = clrTrailerLoad;
        else
            trailerClr = clrLastTrailerLoad;
        end
        circles(trailerLongAxePlt(1,2),trailerLongAxePlt(2,2),0.15/2,'edgecolor',clrLongAxe,'facecolor','none')
        line([trailerLongAxePlt(1,2),xyi(1,i)],[trailerLongAxePlt(2,2),xyi(2,i)],'color',clrLongAxe,'linewidth',1)    
        line(trailerAxePlt(1,:),trailerAxePlt(2,:),'color',clrAxe,'linewidth',2)
        line(trailerWheelLeftPlt(1,:),trailerWheelLeftPlt(2,:),'color',clrWheel,'linewidth',4)
        line(trailerWheelRightPlt(1,:),trailerWheelRightPlt(2,:),'color',clrWheel,'linewidth',4)
        line(trailerLongAxePlt(1,:),trailerLongAxePlt(2,:),'color',clrLongAxe,'linewidth',2)
    end
    %
    if ~noRef
        hold off;
    end    
    %
    xlim([min(coordinates(1,:))-2 max(coordinates(1,:))+2]); ylim([min(coordinates(2,:))-4 max(coordinates(2,:))+4]);
    %
    drawnow limitrate    
end
