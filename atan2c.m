function theta = atan2c( dx_new,dy_new,theta_old )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

Theta_new = atan2(dy_new,dx_new);
Theta_old = atan2(sin(theta_old),cos(theta_old));
d_theta = Theta_new - Theta_old;

if d_theta >= pi
    d_Theta = d_theta - 2*pi;
elseif d_theta < -pi
    d_Theta = d_theta + 2*pi;
else
    d_Theta = d_theta;
end

theta = theta_old + d_Theta;

end

