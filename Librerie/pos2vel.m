function vel_set = pos2vel(err_pos);

segno = sign(err_pos);
err_pos = err_pos * segno;
vel2 = 5;
vel1 = 0.5;
dist2 = 5;
dist1 = 2;

% solo un gradino...
dist1 = 1;
vel1 = vel2/dist2*dist1;


if err_pos>dist2
    vel_set = vel2;
elseif err_pos>dist1
    vel_set = vel1 + (vel2-vel1)/(dist2-dist1)*(err_pos-dist1);
else
    vel_set = vel1/dist1*err_pos;
end

vel_set = vel_set*segno;

end