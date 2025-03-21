clear all
close all
clc

import casadi.*
% *************************************************************************
% General Parameters:
% ************************************************************************* 
% Convert Latitude and Longitude to meters ________________________________
origin                      = [-33.034115, -71.592205];         % Definir el origen para la conversión (latlon2xy)
wf                          = 0.43;
wr                          = 0.32;
l                           = 0.475;

bag                     = rosbag('terraza_27.bag');
gps0_data               = select(bag, 'Topic', '/gps0');    % Pay attention to GPS numbering
gps1_data               = select(bag, 'Topic', '/gps1');
gps2_data               = select(bag, 'Topic', '/gps2');
gps3_data               = select(bag, 'Topic', '/gps3');
rtk_data                = select(bag, 'Topic', '/rtk');

husky_cmdVel            = select(bag, 'Topic','/husky_velocity_controller/cmd_vel');
husky_odomVel           = select(bag, 'Topic','/husky_velocity_controller/odom');
husky_prmsVel           = select(bag, 'Topic','/husky_velocity_controller/parameter_descriptions');

imu2                    = select(bag, 'Topic', '/imu2'); %/imu2
imu3                    = select(bag, 'Topic', '/imu3'); %/imu3

vecINS                  = select(bag, 'Topic', '/vectornav/INS');
vecOdom                 = select(bag, 'Topic', '/vectornav/Odom');

for i=1:1
    vecOdomData = readMessages(vecOdom,i);
    linVelZ     = vecOdomData{1,1}.Twist.Twist.Linear.Z;
    linVel      = sqrt(vecOdomData{1,1}.Twist.Twist.Linear.X^2+...
                    vecOdomData{1,1}.Twist.Twist.Linear.Y^2)*sign(vecOdomData{1,1}.Twist.Twist.Linear.X);
    angVel      = vecOdomData{1,1}.Twist.Twist.Angular.Z;

    huskyOdomVelData = readMessages(husky_odomVel,i);
    eulAngle    = quat2eul([huskyOdomVelData{1,1}.Pose.Pose.Orientation.X,huskyOdomVelData{1,1}.Pose.Pose.Orientation.Y,huskyOdomVelData{1,1}.Pose.Pose.Orientation.Z,huskyOdomVelData{1,1}.Pose.Pose.Orientation.W]);
    huskyAtt    = eulAngle(3);

    imu2Data    = readMessages(imu2,i);
    eulAngle    = quat2eul([imu2Data{1,1}.Orientation.X,imu2Data{1,1}.Orientation.Y,imu2Data{1,1}.Orientation.Z,imu2Data{1,1}.Orientation.W]);
    imu2Att     = eulAngle(3);

    imu3Data    = readMessages(imu3,i);
    eulAngle    = quat2eul([imu3Data{1,1}.Orientation.X,imu3Data{1,1}.Orientation.Y,imu3Data{1,1}.Orientation.Z,imu3Data{1,1}.Orientation.W]);
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
msg0                        = readMessages(gps0_data, 1);
msg1                        = readMessages(gps1_data, 1);
msg2                        = readMessages(gps2_data, 1);
msg3                        = readMessages(gps3_data, 1);
msgRTK                      = readMessages(rtk_data, 1);

[offset_x, offset_y]        = latlon2xy(msgRTK{1,1}.Latitude, msgRTK{1,1}.Longitude, origin(1), origin(2));
rtx_xy                      = [offset_x; offset_y] * 1000;
rtx_xy                      = mtxRot * rtx_xy;
%
[gps0_x, gps0_y]            = latlon2xy(msg0{1,1}.Latitude, msg0{1,1}.Longitude, origin(1), origin(2));
gps0_xy                     = [gps0_x; gps0_y] .* 1000;
gps0_xy                     = mtxRot * gps0_xy;
offset_x0                   = gps0_xy(1)-rtx_xy(1) - l/2;
offset_y0                   = gps0_xy(2)-rtx_xy(2) - wf/2;
cov_gps0_x                  = msg0{1,1}.PositionCovariance(1);
cov_gps0_y                  = msg0{1,1}.PositionCovariance(5);
%
[gps1_x, gps1_y]            = latlon2xy(msg1{1,1}.Latitude, msg1{1,1}.Longitude, origin(1), origin(2));
gps1_xy                     = [gps1_x; gps1_y] .* 1000;
gps1_xy                     = mtxRot * gps1_xy;
offset_x1                   = gps1_xy(1)-rtx_xy(1) - l/2;
offset_y1                   = gps1_xy(2)-rtx_xy(2) + wf/2;
cov_gps1_x                  = msg1{1,1}.PositionCovariance(1);
cov_gps1_y                  = msg1{1,1}.PositionCovariance(5);

%
[gps2_x, gps2_y]            = latlon2xy(msg2{1,1}.Latitude, msg2{1,1}.Longitude, origin(1), origin(2));
gps2_xy                     = [gps2_x; gps2_y] .* 1000;
gps2_xy                     = mtxRot * gps2_xy;
offset_x2                   = gps2_xy(1)-rtx_xy(1) - -l/2;
offset_y2                   = gps2_xy(2)-rtx_xy(2) - wf/2;
cov_gps2_x                  = msg2{1,1}.PositionCovariance(1);
cov_gps2_y                  = msg2{1,1}.PositionCovariance(5);

%
[gps3_x, gps3_y]            = latlon2xy(msg3{1,1}.Latitude, msg3{1,1}.Longitude, origin(1), origin(2));
gps3_xy                     = [gps3_x; gps3_y] .* 1000;
gps3_xy                     = mtxRot * gps3_xy;
offset_x3                   = gps3_xy(1)-rtx_xy(1) + l/2;
offset_y3                   = gps3_xy(2)-rtx_xy(2) + wf/2;
cov_gps3_x                  = msg3{1,1}.PositionCovariance(1);
cov_gps3_y                  = msg3{1,1}.PositionCovariance(5);

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
jac_FNt                     = casadi.Function('Func_D', {q, u}, {x_rk4.jacobian(q)}, {'x', 'u'}, {'D'});


% *************************************************************************
% Initial orientation _____________________________________________________
% *************************************************************************
imu2Data                    = readMessages(imu3,1); % imu2 or imu3
eulAngle                    = quat2eul([imu2Data{1,1}.Orientation.X,imu2Data{1,1}.Orientation.Y,imu2Data{1,1}.Orientation.Z,imu2Data{1,1}.Orientation.W]);
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

msg0                        = readMessages(gps0_data, 1);    
[gps0_x, gps0_y]            = latlon2xy(msg0{1,1}.Latitude, msg0{1,1}.Longitude, origin(1), origin(2));
gps0_x                      = gps0_x*1000;
gps0_y                      = gps0_y*1000;
msg1                        = readMessages(gps1_data, 1);    
[gps1_x, gps1_y]            = latlon2xy(msg1{1,1}.Latitude, msg1{1,1}.Longitude, origin(1), origin(2));
gps1_x                      = gps1_x*1000;
gps1_y                      = gps1_y*1000;
msg2                        = readMessages(gps2_data, 1);    
[gps2_x, gps2_y]            = latlon2xy(msg2{1,1}.Latitude, msg2{1,1}.Longitude, origin(1), origin(2));
gps2_x                      = gps2_x*1000;
gps2_y                      = gps2_y*1000;
msg3                        = readMessages(gps3_data, 1);    
[gps3_x, gps3_y]            = latlon2xy(msg3{1,1}.Latitude, msg3{1,1}.Longitude, origin(1), origin(2));
gps3_x                      = gps3_x*1000;
gps3_y                      = gps3_y*1000;
% Fill mhe horion with good intitial estimates ----------------------------
msgRTK                      = readMessages(rtk_data, 1);    
[gpsRTK_x, gpsRTK_y]        = latlon2xy(msgRTK{1,1}.Latitude, msgRTK{1,1}.Longitude, origin(1), origin(2));
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
setMtxW(mhe4gps,diag([10 1 1]));
setMtxR(mhe4gps,diag([100 0.1 0.1 0.1 0.1 10 10 10 10]));
setPrmsGPSpos(mhe4gps,[wf, wr, l]); % wf, wr, l

for i=1:Ne
    updateMeasurement(mhe4gps, [imu3Att(1); gps0_x-offset_x0; gps0_y-offset_y0;...
        gps1_x-offset_x1; gps1_y-offset_y1; gps2_x-offset_x2; gps2_y-offset_y2; gps3_x-offset_x3; gps3_y-offset_y3] );
    updateInput(mhe4gps,zeros(nu,1));       
end
updateMeasurement(mhe4gps, [imu3Att(1); gps0_x-offset_x0; gps0_y-offset_y0;...
        gps1_x-offset_x1; gps1_y-offset_y1; gps2_x-offset_x2; gps2_y-offset_y2; gps3_x-offset_x3; gps3_y-offset_y3] );
alignFactor4gps             = [0; 2.8; 0.096];


% *************************************************************************
% Init EKF  ___________________________________________________________
% *************************************************************************

Qk                          = diag([0.0001 0.0001 0.0001])*50; %10
Rk                          = diag([0.01, 0.053125, 0.053125]); %1

efkgps                      = ekfMemNic(Ts, Qk, Rk, nq, ny4gps);
set_x0bar(efkgps, x0bar);

vecOdomData = readMessages(vecOdom, i);
linVelX = vecOdomData{1,1}.Twist.Twist.Linear.X;
linVelY = vecOdomData{1,1}.Twist.Twist.Linear.Y;
huskyLinVel = sqrt(linVelX^2 + linVelY^2) * sign(linVelX);
huskyAngVel = vecOdomData{1,1}.Twist.Twist.Angular.Z;

predict(efkgps, FNt, jac_FNt, [huskyAngVel,huskyLinVel]);
update(efkgps, [imu3Att(1); mean([gps0_x-offset_x0;gps1_x-offset_x1;gps2_x-offset_x2;gps3_x-offset_x3]); ...
     mean([gps0_y-offset_y0;gps1_y-offset_y1;  gps2_y-offset_y2;  gps3_y-offset_y3])], jac_hx );

alignFactorefk              = alignFactor4gps;

% *************************************************************************
% Init some variables _____________________________________________________
% *************************************************************************
attitudeOld                 = 0;
correctFactor               = 0;
estimatedPos4gps            = [];
estimatedPosefk             = [];
groundThruth                = [];


% *************************************************************************
% Perform Simulation/Field experiments
% *************************************************************************

%initial message
data_init = 1;

% Determine the number of messages in each topic
num_msgs = min([height(gps0_data.MessageList), height(gps1_data.MessageList), height(gps2_data.MessageList), height(gps3_data.MessageList), height(rtk_data.MessageList)]);
num_msgs= min(num_msgs, 950);

% Pre-read all GPS and RTK messages dynamically
gps0_msgs = readMessages(gps0_data, data_init:num_msgs);
gps1_msgs = readMessages(gps1_data, data_init:num_msgs);
gps2_msgs = readMessages(gps2_data, data_init:num_msgs);
gps3_msgs = readMessages(gps3_data, data_init:num_msgs);
rtk_msgs  = readMessages(rtk_data, data_init:num_msgs);

% Preallocate arrays for efficiency
correctedAtt  = zeros(1, num_msgs-data_init+1);
velocities  = zeros(2, num_msgs-data_init+1);
error4gps = zeros(1, num_msgs-data_init+1);
errorEKF = zeros(1, num_msgs-data_init+1);
groundThruth = zeros(2, num_msgs-data_init+1);
dataGps0 = zeros(2, num_msgs-data_init+1);
dataGps1 = zeros(2, num_msgs-data_init+1);
dataGps2 = zeros(2, num_msgs-data_init+1);
dataGps3 = zeros(2, num_msgs-data_init+1);
tiempos = zeros(1, num_msgs-data_init+1);

for i = 1:num_msgs-data_init+1
    % *********************************************************************
    % Read Sensors and save ground truth __________________________________
    % *********************************************************************
    tic;
    msg0 = gps0_msgs{i};
    msg1 = gps1_msgs{i};
    msg2 = gps2_msgs{i};
    msg3 = gps3_msgs{i};
    msgRTK = rtk_msgs{i};

    % Save Ground truth ___________________________________________________
    [rtk_x, rtk_y]      = latlon2xy(msgRTK.Latitude, msgRTK.Longitude, origin(1), origin(2));
    rtk_xy              = [rtk_x; rtk_y] .* 1000;
    rtk_xy              = mtxRot * rtk_xy;
    rtk_x               = rtk_xy(1);
    rtk_y               = rtk_xy(2);
    groundThruth        = [groundThruth, [rtk_x; rtk_y]];

    % Process GPS data ____________________________________________________
    [gps0_x, gps0_y] = latlon2xy(msg0.Latitude, msg0.Longitude, origin(1), origin(2));
    gps0_xy = [gps0_x; gps0_y] .* 1000;
    gps0_xy = mtxRot * gps0_xy - [offset_x0; offset_y0];
    dataGps0(:, i) = gps0_xy;
    cov_gps0_x = msg0.PositionCovariance(1);
    cov_gps0_y = msg0.PositionCovariance(5);

    [gps1_x, gps1_y] = latlon2xy(msg1.Latitude, msg1.Longitude, origin(1), origin(2));
    gps1_xy = [gps1_x; gps1_y] .* 1000;
    gps1_xy = mtxRot * gps1_xy - [offset_x1; offset_y1];
    dataGps1(:, i) = gps1_xy;
    cov_gps1_x = msg1.PositionCovariance(1);
    cov_gps1_y = msg1.PositionCovariance(5);

    [gps2_x, gps2_y] = latlon2xy(msg2.Latitude, msg2.Longitude, origin(1), origin(2));
    gps2_xy = [gps2_x; gps2_y] .* 1000;
    gps2_xy = mtxRot * gps2_xy - [offset_x2; offset_y2];
    dataGps2(:, i) = gps2_xy;
    cov_gps2_x = msg2.PositionCovariance(1);
    cov_gps2_y = msg2.PositionCovariance(5);

    [gps3_x, gps3_y] = latlon2xy(msg3.Latitude, msg3.Longitude, origin(1), origin(2));
    gps3_xy = [gps3_x; gps3_y] .* 1000;
    gps3_xy = mtxRot * gps3_xy - [offset_x3; offset_y3];
    dataGps3(:, i) = gps3_xy;
    cov_gps3_x = msg3.PositionCovariance(1);
    cov_gps3_y = msg3.PositionCovariance(5);

    % Read attitude from Vectornav2 _______________________________________
    imu2Data = readMessages(imu3, i); % imu2 or imu3
    eulAngle = quat2eul([imu2Data{1,1}.Orientation.X, imu2Data{1,1}.Orientation.Y, imu2Data{1,1}.Orientation.Z, imu2Data{1,1}.Orientation.W]);
    th0 = -eulAngle(3);

    % Unwrap attitude _____________________________________________________
    delta = th0 - attitudeOld;
    if delta > pi
        correctFactor = correctFactor - 2 * pi;
    elseif delta < -pi
        correctFactor = correctFactor + 2 * pi;
    end
    attitudeOld = th0;
    attitude = th0 + correctFactor + attInit;
    correctedAtt(i) = attitude;

    % Read velocities _____________________________________________________
    vecOdomData = readMessages(vecOdom, i);
    linVelX = vecOdomData{1,1}.Twist.Twist.Linear.X;
    linVelY = vecOdomData{1,1}.Twist.Twist.Linear.Y;
    huskyLinVel = sqrt(linVelX^2 + linVelY^2) * sign(linVelX);
    huskyAngVel = vecOdomData{1,1}.Twist.Twist.Angular.Z;
    velocities(:, i) = [huskyAngVel; huskyLinVel];

    % *********************************************************************
    % Update measurements to mhe4gps and solve estimation _________________
    % *********************************************************************
    pos0 = dataGps0(:, i);
    pos1 = dataGps1(:, i);
    pos2 = dataGps2(:, i);
    pos3 = dataGps3(:, i);

    
    updateMeasurement(mhe4gps, [correctedAtt(i); pos0; pos1; pos2; pos3]);
    updateInput(mhe4gps, velocities(:, i));
    solve(mhe4gps);
    q_k = mhe4gps.qk + alignFactor4gps;
    estimatedPos4gps = [estimatedPos4gps, q_k];

    %EKF
    predict(efkgps, FNt, jac_FNt, velocities(:, i));
    update(efkgps, [correctedAtt(i);  mean([pos0, pos1, pos2, pos3],2)], jac_hx);
    
    q_ekf = efkgps.Qtraj(:,end) + alignFactorefk;
    estimatedPosefk = [estimatedPosefk, q_ekf];

    error4gps(i) = sqrt((full(q_k(2)) - groundThruth(1, end))^2 + (full(q_k(3)) - groundThruth(2, end))^2);
    errorEKF(i) = sqrt((full(q_ekf(2)) - groundThruth(1, end))^2 + (full(q_ekf(3)) - groundThruth(2, end))^2);
    tiempos(i) = toc;
end
dif_error = errorEKF(:)-error4gps(:);
disp('Error promedio MHE:')
disp(mean(error4gps(50:end)))
disp('Error promedio EKF:')
disp(mean(errorEKF(50:end)))
disp('Error inicial MHE:')
disp((error4gps(1)))
disp('Error inicial EKF:')
disp((errorEKF(1)))

beep;

first_i=20;

figure(1);
hold on;
grid on;
xlim([-18 22]);
ylim([-15 25]);
title('Datos GPS');
xlabel('X [m]');
ylabel('Y [m]');
daspect([1 1 1]);

% Graficar con DisplayName
plot(estimatedPos4gps(2, end), estimatedPos4gps(3, end), 'o', 'Color', 'r', 'MarkerSize', 7, 'MarkerFaceColor', 'r', 'DisplayName', 'Estimación MHE');
plot(groundThruth(1, end), groundThruth(2, end), 'o', 'Color', 'b', 'MarkerSize', 9, 'DisplayName', 'Ground Truth');
plot(full(estimatedPosefk(2, end)), full(estimatedPosefk(3, end)), 'o', 'Color', 'm', 'MarkerSize', 7, 'MarkerFaceColor', 'm', 'DisplayName', 'Estimación EKF');

% Graficar trayectorias con DisplayName
plot(estimatedPos4gps(2, end-i+first_i:end), estimatedPos4gps(3, end-i+first_i:end), 'r', 'LineWidth', 1.5, 'DisplayName', 'Trayectoria MHE');
plot(groundThruth(1, end-i+first_i:end), groundThruth(2, end-i+first_i:end), 'b', 'LineWidth', 1.5, 'DisplayName', 'Trayectoria Ground Truth');
plot(full(estimatedPosefk(2, end-i+first_i:end)), full(estimatedPosefk(3, end-i+first_i:end)), 'm', 'LineWidth', 1.5, 'DisplayName', 'Trayectoria EKF');

% Mostrar leyenda
legend('show', 'Location', 'best');
hold off;
drawnow;


figure(2);
hold on;
grid on;
title('Error de Posición');
xlabel('Iteración');
ylabel('Error [m]');
ylim([0 6]);

% Graficar errores con DisplayName
plot(error4gps(1+first_i-1:i), 'r', 'LineWidth', 1.5, 'DisplayName', 'Error MHE');
plot(errorEKF(1+first_i-1:i), 'm', 'LineWidth', 1.5, 'DisplayName', 'Error EKF');

% Mostrar leyenda
legend('show', 'Location', 'best');
hold off;
drawnow;

