% *************************************************************************
% MPC and MHE implementation for a differential drive robot
% EKF comparison
% Simulates a significant state error in the first third of the trajectory
% *************************************************************************
clear all
close all
clc

import casadi.*
% *************************************************************************
% General Parameters:
% ************************************************************************* 
SIMULATION                  =  true;
ROSBAGFILE                  = true;
MHE_4GPS                    = true;
% Convert Latitude and Longitude to meters ________________________________
origin                      = [-33.034115, -71.592205];         % Definir el origen para la conversión (latlon2xy)
wf                          = 0.43;
wr                          = 0.32;
l                           = 0.475;
%
if SIMULATION
    if ROSBAGFILE
        bag                     = rosbag('terraza_21.bag');
        gps0_data               = select(bag, 'Topic', '/gps0');    % Pay attention to GPS numbering
        gps1_data               = select(bag, 'Topic', '/gps1');
        gps2_data               = select(bag, 'Topic', '/gps2');
        gps3_data               = select(bag, 'Topic', '/gps3');
        rtk_data                = select(bag, 'Topic', '/rtk');
        
        husky_cmdVel            = select(bag, 'Topic','/husky_velocity_controller/cmd_vel');
        husky_odomVel           = select(bag, 'Topic','/husky_velocity_controller/odom');
        husky_prmsVel           = select(bag, 'Topic','/husky_velocity_controller/parameter_descriptions');
        
        imu2                    = select(bag, 'Topic', '/imu2');
        imu3                    = select(bag, 'Topic', '/imu3');
        
        vecINS                  = select(bag, 'Topic', '/vectornav/INS');
        vecOdom                 = select(bag, 'Topic', '/vectornav/Odom');
    end
else
    rosshutdown;
    pause(5);
    rosinit;
    gps0_data       = rossubscriber('/gps0', 'DataFormat', 'struct');
    gps1_data       = rossubscriber('/gps1', 'DataFormat', 'struct');
    gps2_data       = rossubscriber('/gps2', 'DataFormat', 'struct');
    gps3_data       = rossubscriber('/gps3', 'DataFormat', 'struct');
    rtk_data        = rossubscriber('/rtk', 'DataFormat', 'struct');

    [pub, msg]      = rospublisher('/cmd_vel_aux','geometry_msgs/Twist');
    
    husky_cmdVel    = rossubscriber('/husky_velocity_controller/cmd_vel', 'DataFormat', 'struct');
    husky_odomVel   = rossubscriber('/husky_velocity_controller/odom', 'DataFormat', 'struct');
    husky_prmsVel   = rossubscriber('/husky_velocity_controller/parameter_descriptions', 'DataFormat', 'struct');
    
    imu2            = rossubscriber('/imu2', 'DataFormat', 'struct');
    imu3            = rossubscriber('/imu3', 'DataFormat', 'struct');
    
    vecOdom         = rossubscriber('/vectornav/Odom', 'DataFormat', 'struct');
end

for i=1:1
    vecOdomData = receive(vecOdom,i);
    linVelX     = vecOdomData.Twist.Twist.Linear.X;
    linVelY     = vecOdomData.Twist.Twist.Linear.Y;
    linVelZ     = vecOdomData.Twist.Twist.Linear.Z;
    linVel      = sqrt(vecOdomData.Twist.Twist.Linear.X^2+...
                    vecOdomData.Twist.Twist.Linear.Y^2)*sign(vecOdomData.Twist.Twist.Linear.X);
    angVel      = vecOdomData.Twist.Twist.Angular.Z;

    huskyOdomVelData = receive(husky_odomVel,i);
    huskyLinVel = huskyOdomVelData.Twist.Twist.Linear.X;
    huskyAngVel = huskyOdomVelData.Twist.Twist.Angular.Z;   
    eulAngle    = quat2eul([huskyOdomVelData.Pose.Pose.Orientation.X,huskyOdomVelData.Pose.Pose.Orientation.Y,huskyOdomVelData.Pose.Pose.Orientation.Z,huskyOdomVelData.Pose.Pose.Orientation.W]);
    huskyAtt    = eulAngle(3);

    imu2Data    = receive(imu2,i);
    eulAngle    = quat2eul([imu2Data.Orientation.X,imu2Data.Orientation.Y,imu2Data.Orientation.Z,imu2Data.Orientation.W]);
    imu2Att     = eulAngle(3);

    imu3Data    = receive(imu3,i);
    eulAngle    = quat2eul([imu3Data.Orientation.X,imu3Data.Orientation.Y,imu3Data.Orientation.Z,imu3Data.Orientation.W]);
    imu3Att     = eulAngle(3);    
end
% *************************************************************************
% Compute rotation matrix to navigate in my local reference frame _________
% *************************************************************************
sdpvar a11 a12 a21 a22;
sdpvar x1bar y1bar x2bar y2bar;
% AZOTEA AC3E *************************************************************
lat1                        = -33.034213;           % hand coded value, measurement from rtk
long1                       = -71.592168;           % hand coded value, measurement from rtk
[x1, y1]                    = latlon2xy(lat1, long1, origin(1), origin(2));
x1                          = x1*1000;
y1                          = y1*1000;
x                           = norm([x1 y1]);
%
lat2                        = -33.034088;          % hand coded value, measurement from rtk
long2                       = -71.59211333;        % hand coded value, measurement from rtk
[x2, y2]                    = latlon2xy(lat2, long2, origin(1), origin(2));
x2                          = x2*1000;
y2                          = y2*1000;
y                           = norm([x2 y2]);
% With the "origin" and a point alongside each axe, compute the mtx
A                           = [a11 a12; a21 a22];
v                           = [x1; y1; x2; y2];
b                           = [x1bar; y1bar; x2bar; y2bar];
Constraints                 = [[A zeros(2); zeros(2) A]*v - b == zeros(4,1); x1bar*x2bar + y1bar*y2bar == 0];
%
obj                         = (x1bar - x)^2 + (y2bar - y)^2;
optimize(Constraints, obj);
mtxRot                      = value(A);
% *************************************************************************
% Compute offset for each low-cost GPS ____________________________________
% *************************************************************************
msg0                        = receive(gps0_data, 1);
msg1                        = receive(gps1_data, 1);
msg2                        = receive(gps2_data, 1);
msg3                        = receive(gps3_data, 1);
msgRTK                      = receive(rtk_data, 1);

[offset_x, offset_y]        = latlon2xy(msgRTK.Latitude, msgRTK.Longitude, origin(1), origin(2));
rtx_xy                      = [offset_x; offset_y] * 1000;
rtx_xy                      = mtxRot * rtx_xy;
%
[gps0_x, gps0_y]            = latlon2xy(msg0.Latitude, msg0.Longitude, origin(1), origin(2));
gps0_xy                     = [gps0_x; gps0_y] .* 1000;
gps0_xy                     = mtxRot * gps0_xy;
offset_x0                   = gps0_xy(1)-rtx_xy(1) - l/2;
offset_y0                   = gps0_xy(2)-rtx_xy(2) - wf/2;
%
[gps1_x, gps1_y]            = latlon2xy(msg1.Latitude, msg1.Longitude, origin(1), origin(2));
gps1_xy                     = [gps1_x; gps1_y] .* 1000;
gps1_xy                     = mtxRot * gps1_xy;
offset_x1                   = gps1_xy(1)-rtx_xy(1) - l/2;
offset_y1                   = gps1_xy(2)-rtx_xy(2) + wf/2;
%
[gps2_x, gps2_y]            = latlon2xy(msg2.Latitude, msg2.Longitude, origin(1), origin(2));
gps2_xy                     = [gps2_x; gps2_y] .* 1000;
gps2_xy                     = mtxRot * gps2_xy;
offset_x2                   = gps2_xy(1)-rtx_xy(1) - -l/2;
offset_y2                   = gps2_xy(2)-rtx_xy(2) - wf/2;
%
[gps3_x, gps3_y]            = latlon2xy(msg3.Latitude, msg3.Longitude, origin(1), origin(2));
gps3_xy                     = [gps3_x; gps3_y] .* 1000;
gps3_xy                     = mtxRot * gps3_xy;
offset_x3                   = gps3_xy(1)-rtx_xy(1) + l/2;
offset_y3                   = gps3_xy(2)-rtx_xy(2) + wf/2;
% *************************************************************************
% Kinematic model _________________________________________________________
% *************************************************************************
nq                          = 3;
nu                          = 2;
ny4gps                      = 1+2*4;
Ts                          = 0.2;
q                           = casadi.MX.sym('q',nq);
u                           = casadi.MX.sym('u',nu);
f_rhs                       = [     u(1);...
                                cos(q(1)) * u(2);...
                                sin(q(1)) * u(2) ];

f                           = casadi.Function('f_rhs', {q,u}, {f_rhs});    
opts                        = struct('main',true,'mex',true);
f.generate('f.c',opts)
mex f.c -largeArrayDims;
% Output of the system ----------------------------------------------------
h_rhs                       = q;
h                           = casadi.Function('h', {q}, {h_rhs});
% compute jacobians -------------------------------------------------------
jac_fx                      = casadi.Function('Func_A', {q, u}, {f_rhs.jacobian(q)}, {'x', 'u'}, {'A'});
jac_fu                      = casadi.Function('Func_B', {q, u}, {f_rhs.jacobian(u)}, {'x', 'u'}, {'B'});
jac_hx                      = casadi.Function('Func_C', {q}, {h_rhs.jacobian(q)}, {'x'}, {'C'});
% Integrator --------------------------------------------------------------
k1                          = f(q, u);
k2                          = f(q + Ts / 2 * k1, u);
k3                          = f(q + Ts / 2 * k2, u);
k4                          = f(q + Ts * k3, u);
x_rk4                       = q + Ts / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
FNt                         = casadi.Function('FNt', {q, u}, {x_rk4});
% *************************************************************************
% Initial orientation _____________________________________________________
% *************************************************************************
imu2Data                    = receive(imu3,1); % imu2 or imu3
eulAngle                    = quat2eul([imu2Data.Orientation.X,imu2Data.Orientation.Y,imu2Data.Orientation.Z,imu2Data.Orientation.W]);
attInit                     = eulAngle(3);
% *************************************************************************
% Init MHE with 4 GPSs ____________________________________________________
% *************************************************************************
dims                        = {};
dims.nq                     = nq;
dims.nu                     = nu;
dims.ny                     = ny4gps;
boxConst                    = [];
Ne                          = 15;
Nt                          = 0;
% _________________________________________________________________________
mhe4gps                     = mheMemNic(Ne,Nt,FNt,h,dims,boxConst,Ts);
msg0                        = receive(gps0_data, 1);    
[gps0_x, gps0_y]            = latlon2xy(msg0.Latitude, msg0.Longitude, origin(1), origin(2));
gps0_x                      = gps0_x*1000;
gps0_y                      = gps0_y*1000;
msg1                        = receive(gps1_data, 1);    
[gps1_x, gps1_y]            = latlon2xy(msg1.Latitude, msg1.Longitude, origin(1), origin(2));
gps1_x                      = gps1_x*1000;
gps1_y                      = gps1_y*1000;
msg2                        = receive(gps2_data, 1);    
[gps2_x, gps2_y]            = latlon2xy(msg2.Latitude, msg2.Longitude, origin(1), origin(2));
gps2_x                      = gps2_x*1000;
gps2_y                      = gps2_y*1000;
msg3                        = receive(gps3_data, 1);    
[gps3_x, gps3_y]            = latlon2xy(msg3.Latitude, msg3.Longitude, origin(1), origin(2));
gps3_x                      = gps3_x*1000;
gps3_y                      = gps3_y*1000;
% Fill mhe horion with good intitial estimates ----------------------------
msgRTK                      = receive(rtk_data, 1);    
[gpsRTK_x, gpsRTK_y]        = latlon2xy(msgRTK.Latitude, msgRTK.Longitude, origin(1), origin(2));
gpsRTK_x                    = gpsRTK_x*1000;
gpsRTK_y                    = gpsRTK_y*1000;
gpsRTK                      = mtxRot * [gpsRTK_x; gpsRTK_y];
% *************************************************************************
% Initial condition x0 ____________________________________________________
% *************************************************************************
x0bar                       = [imu3Att(1) - attInit; gpsRTK(1); gpsRTK(2)]; % substract IMU's initial value (vehicle aligned with x axe)
set_x0bar(mhe4gps,x0bar);
setJacobians(mhe4gps,jac_fx,jac_fu,jac_hx);
set_cPrm(mhe4gps,0.15);
set_alpha(mhe4gps,1);
setMtxW(mhe4gps,eye(3));
setMtxR(mhe4gps,diag([10 1 1 1 1 6 6 6 6]));
setPrmsGPSpos(mhe4gps,[wf, wr, l]); % wf, wr, l

for i=1:Ne
    updateMeasurement(mhe4gps, [imu3Att(1); gps0_x-offset_x0; gps0_y-offset_y0;...
        gps1_x-offset_x1; gps1_y-offset_y1; gps2_x-offset_x2; gps2_y-offset_y2; gps3_x-offset_x3; gps3_y-offset_y3] );
    updateInput(mhe4gps,zeros(nu,1));       
end
updateMeasurement(mhe4gps, [imu3Att(1); gps0_x-offset_x0; gps0_y-offset_y0;...
        gps1_x-offset_x1; gps1_y-offset_y1; gps2_x-offset_x2; gps2_y-offset_y2; gps3_x-offset_x3; gps3_y-offset_y3] );
alignFactor4gps             = [0; 3; -1.1];
% *************************************************************************
% Init MHE Loss ___________________________________________________________
% *************************************************************************
dims                        = {};
dims.nq                     = nq;
dims.nu                     = nu;
dims.ny                     = 3;
mheLoss                     = mheOptiLoss3(Ne,Nt,FNt,h,dims,boxConst,Ts);
set_x0bar(mheLoss,x0bar);
setJacobians(mheLoss,jac_fx,jac_fu,jac_hx);
set_alpha(mheLoss,0.8);
set_cPrm(mheLoss,1); % 0.75
setMtxW(mheLoss,diag([0.00001 0.001 0.001])); 
setMtxR(mheLoss,diag([1 100 100]));
for i=1:Ne
    updateMeasurement(mheLoss, full(h(x0bar)));
    updateInput(mheLoss,zeros(nu,1));       
end
updateMeasurement(mheLoss, full(h(x0bar)));
% *************************************************************************
% Create MPC controller with ACADO ________________________________________
% *************************************************************************
Nc                          = 10;
Control a_w a_v;
DifferentialState theta x y w0 v;
f_mpc                       = [ dot(theta) - w0  == 0
                                dot(x) - v*cos(theta) == 0; 
                                dot(y) - v*sin(theta) == 0;
                                dot(w0) - a_w == 0; 
                                dot(v) - a_v == 0];
acadoSet('problemname', 'mpc');
ocpmpc                      = acado.OCP( 0.0, Nc*Ts,Nc );
h                           = [theta, x, y, w0, v];
hN                          = [theta, x, y];
Wmpc                        = acado.BMatrix(eye(length(h)));
WNmpc                       = acado.BMatrix(eye(length(hN)));
%
ocpmpc.minimizeLSQ( Wmpc, h );
ocpmpc.minimizeLSQEndTerm( WNmpc, hN );
ocpmpc.subjectTo( -2*Ts <= a_w <= 2*Ts );
ocpmpc.subjectTo( -0.1*Ts <= a_v <= 0.1*Ts );

ocpmpc.setModel(f_mpc);   % use uncertain model here
%
mpc = acado.OCPexport( ocpmpc );
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'LEVENBERG_MARQUARDT',         1e-12               );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'SPARSE_QP_SOLUTION',          'CONDENSING'        ); % FULL_CONDENSING_N2
mpc.set( 'HOTSTART_QP',                 'YES'               );
mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'       ); % INT_EX_EULER, INT_IRK_GL4, INT_IRK_RIIA5,INT_RK78, INT_RK12 INT_RK23, INT_RK45, INT_RK78, INT_BDF. More details gere: https://acado.sourceforge.net/doc/html/da/d05/classIntegrator.html
mpc.set( 'NUM_INTEGRATOR_STEPS',        (Nc+1)   );
mpc.set( 'QP_SOLVER',                   'QP_QPOASES'    	);
%
mpc.exportCode( 'export_MPC' );    
global ACADO_; %#ok<*TLEV>
copyfile([ACADO_.pwd '/../../external_packages/qpoases'], 'export_MPC/qpoases')    
cd export_MPC
make_acado_solver('../acado_MPCstep')
cd ..
%
input_MPC                   = struct;
Xref                        = [x0bar; 0; 0];
Uref                        = [0 0];
input_MPC.x                 = repmat(Xref',Nc+1,1);
output_MPC.x                = input_MPC.x;
Uref                        = repmat(Uref,Nc,1);
input_MPC.u                 = zeros(size(Uref));
output_MPC.u                = input_MPC.u;
% input_MPC.y                 = repmat(x0bar,Nc,1);
% input_MPC.yN                = x0bar';
input_MPC.W                 = diag([1e-6 1 1 0.1 0.11]);
input_MPC.WN                = 5.*diag([1e-6 1 1]);   
input_MPC.shifting.strategy = 1;   
controls_MPC_w0             = 0;
controls_MPC_v              = 0;
% call controller
input_MPC.y                 = [repmat(x0bar', Nc,1), Uref];    
input_MPC.yN                = x0bar'; % UPDATE REFRENCE

% Solve MPC
input_MPC.x0                = [x0bar; 0; 0]';
output_MPC                  = acado_MPCstep(input_MPC);
% *************************************************************************
% Init some variables _____________________________________________________
% *************************************************************************
attitudeOld                 = 0;
correctFactor               = 0;
%
groundThruth                = [];
estimatedPos4gps            = [];
estimatedPosLoss            = [];
measuredPosLoss             = [];
dataGps0                    = [];
dataGps1                    = [];
dataGps2                    = [];
dataGps3                    = [];
correctedAtt                = [];
attitude                    = 0;
probGpsOutlier              = 20; % in %
timeExecMheLoss             = [];
velocities                  = [];
pathIdx                     = 1;
vref                        = 0.3; % (m/s)
pathRef                     = [];
dt                          = 1e-3;
controls_MPC                = [];
t_mhe4gps                   = [];
t_mheLoss                   = [];
t_mpc                       = [];
t_updtRef                   = [];
t_readSen                   = [];
% *************************************************************************
% Configure plot __________________________________________________________
% *************************************************************************
if SIMULATION
    figure(6);
    subplot(1,2,1)
    hold on;
    grid on;
    xlim([-40 40]);
    ylim([-40 40]);
    title('GPS Data y Contorno de la Terraza');
    xlabel('X (m)');
    ylabel('Y (m)');
    daspect([1 1 1])
    colors = {'r', 'g', 'b', 'm'}; % Definir colores para cada GPS
    subplot(1,2,2)
    hold on;
    grid on;
end
% *************************************************************************
% Perform Simulation/Field experiments
% *************************************************************************
for i = 1:3900%min([height(gps0_data.MessageList),height(gps1_data.MessageList),height(gps2_data.MessageList),height(gps3_data.MessageList),height(rtk_data.MessageList)])
    % *********************************************************************
    % Read Sensors ans save ground truth __________________________________
    % *********************************************************************
    % Read 4 bad GPSs _____________________________________________________
    tic;
    msg0                = receive(gps0_data, i);
    msg1                = receive(gps1_data, i);
    msg2                = receive(gps2_data, i);
    msg3                = receive(gps3_data, i);
    msgRTK              = receive(rtk_data, i);
    % Save Ground truth ___________________________________________________
    [rtk_x, rtk_y]      = latlon2xy(msgRTK.Latitude, msgRTK.Longitude, origin(1), origin(2));
    rtk_xy              = [rtk_x; rtk_y] .* 1000;
    rtk_xy              = mtxRot * rtk_xy;
    rtk_x               = rtk_xy(1);
    rtk_y               = rtk_xy(2);
    groundThruth        = [groundThruth, [rtk_x; rtk_y]];
    [gps0_x, gps0_y]    = latlon2xy(msg0.Latitude, msg0.Longitude, origin(1), origin(2));
    gps0_xy             = [gps0_x; gps0_y] .* 1000;
    gps0_xy             = mtxRot * gps0_xy - [offset_x0; offset_y0];
    gps0_x              = gps0_xy(1);
    gps0_y              = gps0_xy(2);
    dataGps0            = [dataGps0, gps0_xy];
    [gps1_x, gps1_y]    = latlon2xy(msg1.Latitude, msg1.Longitude, origin(1), origin(2));
    gps1_xy             = [gps1_x; gps1_y] .* 1000;
    gps1_xy             = mtxRot * gps1_xy - [offset_x1; offset_y1];
    gps1_x              = gps1_xy(1);
    gps1_y              = gps1_xy(2);
    dataGps1            = [dataGps1, gps1_xy];
    [gps2_x, gps2_y]    = latlon2xy(msg2.Latitude, msg2.Longitude, origin(1), origin(2));
    gps2_xy             = [gps2_x; gps2_y] .* 1000;
    gps2_xy             = mtxRot * gps2_xy - [offset_x2; offset_y2];
    gps2_x              = gps2_xy(1);
    gps2_y              = gps2_xy(2);
    dataGps2            = [dataGps2, gps2_xy];
    [gps3_x, gps3_y]    = latlon2xy(msg3.Latitude, msg3.Longitude, origin(1), origin(2));
    gps3_xy             = [gps3_x; gps3_y] .* 1000;
    gps3_xy             = mtxRot * gps3_xy - [offset_x3; offset_y3];
    gps3_x              = gps3_xy(1);
    gps3_y              = gps3_xy(2);
    dataGps3            = [dataGps3, gps3_xy];
    % Read attitude from Vectornav2 _______________________________________
    imu2Data            = receive(imu3,i); % imu2 or imu3
    eulAngle            = quat2eul([imu2Data.Orientation.X,imu2Data.Orientation.Y,imu2Data.Orientation.Z,imu2Data.Orientation.W]);
    th0                 = -eulAngle(3);
    % Unwrap attitude _____________________________________________________
    delta = th0-attitudeOld;
    if (delta) > pi
        correctFactor = correctFactor - 2*pi;
    elseif (delta) < -pi
        correctFactor = correctFactor + 2*pi;         
    end
    attitudeOld         = th0;
    attitude            = th0 + correctFactor + attInit;
    correctedAtt        = [correctedAtt, attitude];
    % Read velocities _____________________________________________________
    % husky's odometry ----------------------------------------------------
    huskyOdomVelData    = receive(husky_odomVel,i);
    % huskyLinVel         = huskyOdomVelData.Twist.Twist.Linear.X;
    % huskyAngVel         = huskyOdomVelData.Twist.Twist.Angular.Z;
    % % Velocities from Vectornav -----------------------------------------
    vecOdomData         = receive(vecOdom,i);
    linVelX             = [linVelX, vecOdomData.Twist.Twist.Linear.X];
    linVelY             = [linVelY, vecOdomData.Twist.Twist.Linear.Y];
    huskyLinVel         = sqrt(vecOdomData.Twist.Twist.Linear.X^2+...
                            vecOdomData.Twist.Twist.Linear.Y^2)*sign(vecOdomData.Twist.Twist.Linear.X);
    huskyAngVel         = vecOdomData.Twist.Twist.Angular.Z;
    t_readSen           = [t_readSen, toc];
    % *********************************************************************
    % Update measurements to mhe4gps and solve estimation _________________
    % *********************************************************************    
    pos0                = [gps0_x; gps0_y];
    pos1                = [gps1_x; gps1_y];
    pos2                = [gps2_x; gps2_y];
    pos3                = [gps3_x; gps3_y];
    velocities          = [velocities, [huskyAngVel; huskyLinVel]];
    if MHE_4GPS
        updateMeasurement(mhe4gps,[correctedAtt(end); pos0; pos1; pos2; pos3]);
        updateInput(mhe4gps,velocities(:,end));
        tic;
        solve(mhe4gps);
        t_mhe4gps           = [t_mhe4gps, toc];
        q_k                 = mhe4gps.qk + alignFactor4gps;
        estimatedPos4gps    = [estimatedPos4gps, q_k];
    else
    % *********************************************************************
    % Update measurements to mheLoss and solve the estimation problem _____
    % *********************************************************************
        if randi(100) > (100-probGpsOutlier)
            % pos = pos0 + [offset_x0; offset_y0];
            pos = pos1 + [offset_x1*sign(rand*2-1); offset_y1*sign(rand*2-1)];
            % pos = pos2 + [offset_x2; offset_y2];
            % pos = pos3 + [offset_x3; offset_y3];
        else
            pos = groundThruth(:,end);
        end
        measuredPosLoss     = [measuredPosLoss, pos];
        updateMeasurement(mheLoss,[correctedAtt(end); pos]);
        updateInput(mheLoss,velocities(:,end));
        tic;
        solve(mheLoss);
        t_mheLoss           = [t_mheLoss, toc];
        q_k                 = mheLoss.qk;    
        estimatedPosLoss    = [estimatedPosLoss, q_k];
    end
    % *********************************************************************
    % Update reference ____________________________________________________
    % *********************************************************************
    tic;
    [ref, pathIdx]      = genPathRef('rectangular',pathIdx,vref,Ts,dt);
    pathRef             = [pathRef, ref];
    t_updtRef           = [t_updtRef, toc];
    % *********************************************************************
    % Solve Control Problem _______________________________________________
    % *********************************************************************
    uref                = [0; vref];
    input_MPC.y         = [repmat(ref', Nc,1), repmat(uref',Nc,1)];    
    input_MPC.yN        = ref';
    input_MPC.x0        = [estimatedPosLoss(:,end); huskyAngVel; huskyLinVel]';
    tic;
    output_MPC          = acado_MPCstep(input_MPC);
    t_mpc               = [t_mpc, toc];   
    controls_MPC        = [controls_MPC, output_MPC.u(1,:)'];

    input_MPC.x         = output_MPC.x;
    input_MPC.u         = output_MPC.u; 
    if ~SIMULATION
        msg.Linear.X = input_MPC.x(1,5);
        msg.Angular.Z = input_MPC.x(1,4);
        send(pub,msg);
    end    
    % *********************************************************************
    % Wait until Ts _______________________________________________________
    % *********************************************************************
    if ~SIMULATION
        t_acum = t_mheLoss(end) + t_mpc(end) + t_updtRef(end) + t_readSen(end);
        while t_acum < Ts
            tic;
            t_acum = t_acum + toc;
        end
    end
    % *********************************************************************
    % Do some plots _______________________________________________________
    % *********************************************************************
    if SIMULATION
        xMeasured           = [gps0_x, gps1_x, gps2_x, gps3_x];
        yMeasured           = [gps0_y, gps1_y, gps2_y, gps3_y];
        xEstimated          = [mhe4gps.x0Val(end), mhe4gps.x1Val(end), mhe4gps.x2Val(end), mhe4gps.x3Val(end)];
        yEstimated          = [mhe4gps.y0Val(end), mhe4gps.y1Val(end), mhe4gps.y2Val(end), mhe4gps.y3Val(end)];
        subplot(1,2,1)
        cla;
        plot(estimatedPos4gps(2,end), estimatedPos4gps(3,end), 'o', 'Color', 'r', 'MarkerSize',15,'MarkerFaceColor','r');
        plot(estimatedPosLoss(2,end), estimatedPosLoss(3,end), 'o', 'Color', 'b', 'MarkerSize',15,'MarkerFaceColor','b');
        plot(rtk_x, rtk_y, 'o', 'Color', 'k', 'MarkerSize',17);
        subplot(1,2,2); hold on;
        cla;
        % plot(correctedAtt.*180./pi,'b')
        % plot(velocities(1,:).*180./pi,'r')
        plot(pathRef(1,:),pathRef(2,:),'r')
        drawnow;
    end
end




function [ref, pathIndx] = genPathRef(path,pathIdx,vref,Ts,dt)
    if strcmp(path,'infinity')
        correction_x = 6.5;  % correction for the field experiemnts in order to fit the path in my locala reference frame
        correction_y = 4.5;
        a            = 1.75;
        c            = 4;
        b            = 1;
        length_t     = 2*pi/dt;
        if pathIdx >= length_t            
            pathIndx    = length_t;
            i           = length_t-1;
            dx          = (a*sqrt(2).*cos((i+1)*dt))./(sin((i+1)*dt).^2+1) - (a*sqrt(2).*cos(i*dt))./(sin(i*dt).^2+1);
            dy          = (c*sqrt(2).*cos((i+1)*dt).*sin((i+1)*dt))./(sin((i+1)*dt).^2 + b) - (c*sqrt(2).*cos(i*dt).*sin(i*dt))./(sin(i*dt).^2 + b);
            th          = atan2(dy, dx);
            ref         = [a*sqrt(2)+correction_x; correction_y; th];
            return;
        end
        d = 0;
        for i=pathIdx:length_t-1
            dx      = (a*sqrt(2).*cos((i+1)*dt))./(sin((i+1)*dt).^2+1) - (a*sqrt(2).*cos(i*dt))./(sin(i*dt).^2+1);
            dy      = (c*sqrt(2).*cos((i+1)*dt).*sin((i+1)*dt))./(sin((i+1)*dt).^2 + b) - (c*sqrt(2).*cos(i*dt).*sin(i*dt))./(sin(i*dt).^2 + b);
            d       = d + sqrt(dx^2+dy^2);
            velPath = d/Ts;
            th      = atan2(dy, dx);
            if velPath >= vref
                ref = [(a*sqrt(2).*cos((i+1)*dt))./(sin((i+1)*dt).^2+1)+correction_x; (c*sqrt(2).*cos((i+1)*dt).*sin((i+1)*dt))./(sin((i+1)*dt).^2 + b)+correction_y; th];
                pathIndx = i;
                break;
            end
            ref = [a*sqrt(2)+correction_x; correction_y; th];
            pathIndx = length_t;
        end        
    elseif strcmp(path,'rectangular')
        correction_x = 6.5;  % correction for the field experiemnts in order to fit the path in my locala reference frame
        correction_y = 4.5; 
        p = 2.5;
        q = 2;
        length_t     = 2*pi/dt;
        if pathIdx >= length_t            
            pathIndx    = length_t;
            i           = length_t-1;
            dx          = p.*(sqrt(cos((i+1)*dt).*cos((i+1)*dt)).*cos((i+1)*dt) + sqrt(sin((i+1)*dt).*sin((i+1)*dt)).*sin((i+1)*dt)) - p.*(sqrt(cos((i)*dt).*cos((i)*dt)).*cos((i)*dt) + sqrt(sin((i)*dt).*sin((i)*dt)).*sin((i)*dt));
            dy          = q.*(sqrt(cos((i+1)*dt).*cos((i+1)*dt)).*cos((i+1)*dt) - sqrt(sin((i+1)*dt).*sin((i+1)*dt)).*sin((i+1)*dt)) - q.*(sqrt(cos((i)*dt).*cos((i)*dt)).*cos((i)*dt) - sqrt(sin((i)*dt).*sin((i)*dt)).*sin((i)*dt));
            th          = atan2(dy, dx);
            ref         = [p.*(sqrt(cos(2*pi).*cos(2*pi)).*cos(2*pi) + sqrt(sin(2*pi).*sin(2*pi)).*sin(2*pi))+correction_x; q.*(sqrt(cos(2*pi).*cos(2*pi)).*cos(2*pi) - sqrt(sin(2*pi).*sin(2*pi)).*sin(2*pi))+correction_y; th];
            return;
        end
        d = 0;
        for i=pathIdx:length_t-1
            dx          = p.*(sqrt(cos((i+1)*dt).*cos((i+1)*dt)).*cos((i+1)*dt) + sqrt(sin((i+1)*dt).*sin((i+1)*dt)).*sin((i+1)*dt)) - p.*(sqrt(cos((i)*dt).*cos((i)*dt)).*cos((i)*dt) + sqrt(sin((i)*dt).*sin((i)*dt)).*sin((i)*dt));
            dy          = q.*(sqrt(cos((i+1)*dt).*cos((i+1)*dt)).*cos((i+1)*dt) - sqrt(sin((i+1)*dt).*sin((i+1)*dt)).*sin((i+1)*dt)) - q.*(sqrt(cos((i)*dt).*cos((i)*dt)).*cos((i)*dt) - sqrt(sin((i)*dt).*sin((i)*dt)).*sin((i)*dt));
            d       = d + sqrt(dx^2+dy^2);
            velPath = d/Ts;
            th      = atan2(dy, dx);
            if velPath >= vref
                ref = [p.*(sqrt(cos((i+1)*dt).*cos((i+1)*dt)).*cos((i+1)*dt) + sqrt(sin((i+1)*dt).*sin((i+1)*dt)).*sin((i+1)*dt))+correction_x; q.*(sqrt(cos((i+1)*dt).*cos((i+1)*dt)).*cos((i+1)*dt) - sqrt(sin((i+1)*dt).*sin((i+1)*dt)).*sin((i+1)*dt))+correction_y; th];
                pathIndx = i;
                break;
            end

            ref = [p.*(sqrt(cos(2*pi).*cos(2*pi)).*cos(2*pi) + sqrt(sin(2*pi).*sin(2*pi)).*sin(2*pi))+correction_x; q.*(sqrt(cos(2*pi).*cos(2*pi)).*cos(2*pi) - sqrt(sin(2*pi).*sin(2*pi)).*sin(2*pi))+correction_y; th];
            pathIndx = length_t;
        end         
    elseif strcmp(path,'circular')
        a = 2;
        b = 2;
%         b = 0.4;
        t = 0:0.0001:2*pi;        
        x = a*cos(t);
        y = b*sin(t);
        %
        S.path.coorection_x = 5.5;
        S.path.coorection_y = 4.5;
        S.path.coordinates  = [x+S.path.coorection_x;y+S.path.coorection_y];
     elseif strcmp(path,'rect')
        x = 0:0.0001:100;
        y = zeros(1,length(x));
        S.path.coordinates  = [x;y];
    end
end