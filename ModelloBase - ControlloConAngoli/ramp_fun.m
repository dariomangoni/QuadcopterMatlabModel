function [y] = ramp_fun(x)

    dist_incl_max = 10;
    incl_max = sind(15);

    if x>=dist_incl_max
        y = incl_max;
    elseif x<=-dist_incl_max
        y = -incl_max;
    elseif x>=0
        y = incl_max/2*sin(pi*(x-dist_incl_max/2)/dist_incl_max)+incl_max/2;
    else
        y = -incl_max/2*sin(pi*(x-dist_incl_max/2)/dist_incl_max)-incl_max/2;
    end
end

