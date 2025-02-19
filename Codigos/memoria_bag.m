% MPC and MHE implementation for a differential drive robot
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
bag = rosbag('5min.bag');

gps0_data = select(bag, 'Topic', '/gps0');
gps1_data = select(bag, 'Topic', '/gps1');
gps2_data = select(bag, 'Topic', '/gps2');
gps3_data = select(bag, 'Topic', '/gps3');

% Definir el origen para la conversión (usado en grn2eqa)
origin = [-33.033931318599, -71.59195509929];
rotacion = -0.16+pi/2;

R_fun = @(theta) ([cos(theta) -sin(theta); sin(theta) cos(theta)]);
Rot_fun = @(vect) ((R_fun(rotacion)*vect')');
%% --- Agregar contorno de la terraza ---

% Vértices de la terraza (latitud, longitud)
lat_vertices = [ -33.033896785015536, ...
                 -33.033810029854315, ...
                 -33.03375780825845,  ...
                 -33.03374096257619,  ...
                 -33.033721590037615, ...
                 -33.034217694143855, ...
                 -33.034236224293906, ...
                 -33.0341410466636,   ...
                 -33.03412672789615,  ...
                 -33.03404586893048,  ...
                 -33.03405597630525,  ...
                 -33.0339271071901,   ...
                 -33.03391445391645,  ...
                 -33.03382226025864,  ...
                 -33.03383912495935,  ...
                 -33.033863859847884];

lon_vertices = [ -71.5922133239972, ...
                 -71.59221834746305, ...
                 -71.59197320232951, ...
                 -71.59197822579537, ...
                 -71.59186971893298, ...
                 -71.59176724022933, ...
                 -71.5918898127961,  ...
                 -71.59191493012536, ...
                 -71.59185565322831, ...
                 -71.59187474239855, ...
                 -71.5919330146024,  ...
                 -71.59196215070435, ...
                 -71.5919001140145,  ...
                 -71.5919168778208,  ...
                 -71.59201746065855, ...
                 -71.59201477844954];

% Asegurarse de cerrar el contorno (el último punto debe ser igual al primero)
if lat_vertices(1) ~= lat_vertices(end) || lon_vertices(1) ~= lon_vertices(end)
    lat_vertices(end+1) = lat_vertices(1);
    lon_vertices(end+1) = lon_vertices(1);
end

% Convertir los vértices a coordenadas en metros usando grn2eqa de forma vectorizada.
% Se asume que grn2eqa acepta vectores de latitud y longitud y retorna [x, y].
terraza = gr2xy(lat_vertices, lon_vertices, origin);

terraza = Rot_fun(terraza);

x_contour = terraza(:,1);
y_contour = terraza(:,2);

%% --- Plotear datos GPS y el contorno ---
figure(6);
hold on;
grid on;
title('GPS Data y Contorno de la Terraza');
xlabel('X (m)');
ylabel('Y (m)');

% Graficar el contorno de la terraza en negro
plot(x_contour, y_contour, 'k-', 'LineWidth', 2);

% Definir colores para cada GPS
colors = {'r', 'g', 'b', 'm'};

% Plotear datos de GPS
for i = 1:height(gps1_data.MessageList)
    msg0 = readMessages(gps0_data, i);
    msg1 = readMessages(gps1_data, i);
    msg2 = readMessages(gps2_data, i);
    msg3 = readMessages(gps3_data, i);

    gps0 = gr2xy(msg0{1}.Latitude, msg0{1}.Longitude, origin);
    gps1 = gr2xy(msg1{1}.Latitude, msg1{1}.Longitude, origin);
    gps2 = gr2xy(msg2{1}.Latitude, msg2{1}.Longitude, origin);
    gps3 = gr2xy(msg3{1}.Latitude, msg3{1}.Longitude, origin);

    gps0 = Rot_fun(gps0);
    gps1 = Rot_fun(gps1);
    gps2 = Rot_fun(gps2);
    gps3 = Rot_fun(gps3); 

    x = [gps0(1), gps1(1), gps2(1), gps3(1)];
    y = [gps0(2), gps1(2), gps2(2), gps3(2)];
    
    plot(x(1), y(1), 'o', 'Color', colors{1});  % GPS 0
    plot(x(2), y(2), 'o', 'Color', colors{2});  % GPS 1
    plot(x(3), y(3), 'o', 'Color', colors{3});  % GPS 2
    plot(x(4), y(4), 'o', 'Color', colors{4});  % GPS 3
    drawnow;
end




function xy = gr2xy(lat, lon, origin)
    R = 6371000+70; % Radio de la Tierra en metros
    deg2rad = pi/180;
    
    % Diferencias en grados
    dlat = lat - origin(1)*ones(size(lat));
    dlon = lon - origin(2)*ones(size(lon));
    
    % Convertir diferencias a metros
    dx = R * dlon * deg2rad * cos(origin(1) * deg2rad); % Este (x)
    dy = R * dlat * deg2rad;                             % Norte (y)
    
    xy = [dx', dy'];
end