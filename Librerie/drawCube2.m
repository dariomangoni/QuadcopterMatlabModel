function drawCube2 ( origin, dimension, quaternione, color , dt)

scrsz = get(0,'ScreenSize');
figure('OuterPosition',[1 1 scrsz(3) scrsz(4)],'Name','Quadricottero','NumberTitle','off','Tag','ModelloQuadr rosso:non_filtr blu:filtr');
video = VideoWriter('QuadricotteroControllato.avi');
delta_stp = 25;
video.FrameRate = 1/dt/delta_stp;
video.Quality = 50;
open(video); 
axis equal
xlim([min(origin(1,:))-2.5 max(origin(1,:))+2.5])
ylim([min(origin(2,:))-2.5 max(origin(2,:))+2.5])
zlim([min(origin(3,:))-2.5 max(origin(3,:))+2.5])
grid on
set(gca,'CameraPosition',[7 43 27]);
set(gca,'nextplot','replacechildren');
set(gcf,'Renderer','zbuffer');
text = annotation('textbox',[0.8 0 0.1 0.1],'String',['Tempo: ',num2str(0),'s']);

h1=patch([0;1;1;0],zeros(4,1),[0;0;1;1],'w');
h2=patch([0;1;1;0],zeros(4,1),[0;0;1;1],'w');
h3=patch([0;1;1;0],zeros(4,1),[0;0;1;1],'w');
h4=patch([0;1;1;0],zeros(4,1),[0;0;1;1],'w');
h5=patch([0;1;1;0],zeros(4,1),[0;0;1;1],'w');
h6=patch([0;1;1;0],zeros(4,1),[0;0;1;1],'w');
hf=patch([0;1;1;0;5;6;7],zeros(7,1),[0;0;1;1;5;6;7],'w');

set(h1,'edgecolor', color)
set(h2,'edgecolor', color)
set(h3,'edgecolor', color)
set(h4,'edgecolor', color)
set(h5,'edgecolor', color)
set(h6,'edgecolor', color)

set(hf,'edgecolor', color)

for numero_quat = 1:delta_stp:size(quaternione,2)
    
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
    
    quat = quaternione(:,numero_quat);
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
    
    if size(origin,2) == size(quaternione,2)
        numero_pos = numero_quat;
    else
        numero_pos = 1;
    end
        
    x = x + origin(1,numero_pos);
    y = y + origin(2,numero_pos);
    z = z + origin(3,numero_pos);
    
    xf = xf + origin(1,numero_pos);
    yf = yf + origin(2,numero_pos);
    zf = zf + origin(3,numero_pos);
    

        set(h1,'XData',x(:,1));
        set(h1,'YData',y(:,1));
        set(h1,'ZData',z(:,1));
        
        set(h2,'XData',x(:,2));
        set(h2,'YData',y(:,2));
        set(h2,'ZData',z(:,2));
        
        set(h3,'XData',x(:,3));
        set(h3,'YData',y(:,3));
        set(h3,'ZData',z(:,3));
        
        set(h4,'XData',x(:,4));
        set(h4,'YData',y(:,4));
        set(h4,'ZData',z(:,4));
        
        set(h5,'XData',x(:,5));
        set(h5,'YData',y(:,5));
        set(h5,'ZData',z(:,5));
        
        set(h6,'XData',x(:,6));
        set(h6,'YData',y(:,6));
        set(h6,'ZData',z(:,6));
        
        set(hf,'XData',xf);
        set(hf,'YData',yf);
        set(hf,'ZData',zf);
        
        set(text,'String',['Tempo: ',num2str(numero_quat*dt),'s']);
        
        drawnow
        writeVideo(video,getframe(gcf));

end
close(video)
end