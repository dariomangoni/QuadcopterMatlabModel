function drawCube ( origin, dimension, quat, color )
    
x=([0 1 1 0 0 0;...
    1 1 0 0 1 1;...
    1 1 0 0 1 1;...
    0 1 1 0 0 0]   -0.5)*dimension(1);
y=([0 0 1 1 0 0;...
    0 1 1 0 0 0;...
    0 1 1 0 1 1;...
    0 0 1 1 1 1]   -0.5)*dimension(2);
z=([0 0 0 0 0 1;...
    0 0 0 0 0 1;...
    1 1 1 1 0 1;...
    1 1 1 1 0 1]   -0.5)*dimension(3);

xf=([0    ;    0 ; 0.75 ; 0.75 ;  1  ; 0.75 ; 0.75]-0.5)*dimension(1);
yf=([0.25 ; 0.75 ; 0.75 ; 1    ; 0.5 ; 0    ; 0.25]-0.5)*dimension(2);
zf=([1;1;1;1;1;1;1]-0.5)*dimension(3);

rotation_matrix=2*[0.5-quat(3)^2-quat(4)^2,         quat(2)*quat(3)-quat(1)*quat(4), quat(2)*quat(4)+quat(1)*quat(3);...
                   quat(2)*quat(3)+quat(1)*quat(4), 0.5-quat(2)^2-quat(4)^2,         quat(3)*quat(4)-quat(1)*quat(2);...
                   quat(2)*quat(4)-quat(1)*quat(3), quat(3)*quat(4)+quat(1)*quat(2), 0.5-quat(2)^2-quat(3)^2       ];

for colonna = 1:6
    for riga = 1:4
        vettore = rotation_matrix * [x(riga,colonna);...
                                     y(riga,colonna);...
                                     z(riga,colonna)];
        x(riga,colonna)=vettore(1);
        y(riga,colonna)=vettore(2);
        z(riga,colonna)=vettore(3);
    end
end

for riga = 1: size(xf,1)
     vettore = rotation_matrix * [xf(riga);...
                                  yf(riga);...
                                  zf(riga)];
        xf(riga)=vettore(1);
        yf(riga)=vettore(2);
        zf(riga)=vettore(3);
end


x = x + origin(1);
y = y + origin(2);
z = z + origin(3);

xf = xf + origin(1);
yf = yf + origin(2);
zf = zf + origin(3);

h=patch(xf,yf,zf,'w');

%% Parallelepipedo
for i=1:size(x,2)
    h=patch(x(:,i),y(:,i),z(:,i),'w');
    set(h,'edgecolor', color)
end



end