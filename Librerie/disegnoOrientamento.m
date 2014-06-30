function disegnoOrientamento (quaternioni, disegnanuovo)

stp_fin = size(quaternioni,2);
scrsz = get(0,'ScreenSize');
vettore_prova=[0;0;1];
disegno_vettore=zeros(3,stp_fin);
for stp=1:stp_fin
    quat = quaternioni(:,stp);
    rotation_matrix=2*[0.5-quat(3)^2-quat(4)^2,         quat(2)*quat(3)-quat(1)*quat(4), quat(2)*quat(4)+quat(1)*quat(3);...
                       quat(2)*quat(3)+quat(1)*quat(4), 0.5-quat(2)^2-quat(4)^2,         quat(3)*quat(4)-quat(1)*quat(2);...
                       quat(2)*quat(4)-quat(1)*quat(3), quat(3)*quat(4)+quat(1)*quat(2), 0.5-quat(2)^2-quat(3)^2       ];
    disegno_vettore(:,stp)=rotation_matrix*vettore_prova;
end

if disegnanuovo == true
    figure('OuterPosition',[scrsz(3)/2+12 scrsz(4)/2 scrsz(3)/2-12 scrsz(4)/2],'Name','Orientamento','NumberTitle','off');
end
hold on
grid on
axis equal
xlim([-1 1])
ylim([-1 1])
zlim([-1 1])
xlabel('asse X')
ylabel('asse Y')
zlabel('asse Z')

set(gca,'CameraPosition',[1.5 2 1.7]);
line(disegno_vettore(1,:),disegno_vettore(2,:),disegno_vettore(3,:),'Color',rand(1,3),'Marker','.')
end