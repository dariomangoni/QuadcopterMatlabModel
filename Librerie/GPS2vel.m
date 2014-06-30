function [vel] = GPS2vel (gps_vel)

vel_abs = gps_vel(1)/3.6; % in metri al secondo
vel_ang = gps_vel(2)*pi/180;

vel = zeros(2,1);
vel(1) = vel_abs * cos(vel_ang);
vel(2) = vel_abs * sin(vel_ang);

end