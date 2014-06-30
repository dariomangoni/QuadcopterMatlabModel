clear all
close all
clc

err_pos = -10:0.05:10;
vel_set = zeros(size(err_pos));

for s = 1:numel(err_pos)
    vel_set(s) = pos2vel(err_pos(s));
end

plot(err_pos,vel_set,'b');