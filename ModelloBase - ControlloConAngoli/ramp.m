% rampa
clear all
clc

dist_incl_max = 5;
incl_max = sind(22.5);

stp = 0;
for x = -dist_incl_max-1:0.05:dist_incl_max+1
    stp = stp+1;
    if x>=dist_incl_max
        y(stp) = incl_max;
    elseif x<=-dist_incl_max
        y(stp) = -incl_max;
    elseif x>=0
        y(stp) = incl_max/2*sin(pi*(x-dist_incl_max/2)/dist_incl_max)+incl_max/2;
    elseif x<=0
        y(stp) = -incl_max/2*sin(pi*(x-dist_incl_max/2)/dist_incl_max)-incl_max/2;
    end
    x_table(stp) = x;
end
plot(x_table,y);
line([-dist_incl_max dist_incl_max],[-incl_max incl_max],'Color','red')
