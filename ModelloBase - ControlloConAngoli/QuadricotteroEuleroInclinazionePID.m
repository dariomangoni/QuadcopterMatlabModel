clear all
clc

raggiunto=0;

stp=2;
dt=0.001;
stp_fin = 25000;


%% Allocazione matrici
%Controllo
err_x=zeros(stp_fin,1);
err_y=zeros(stp_fin,1);
err_z=zeros(stp_fin,1);
err_x_m=zeros(stp_fin,1);
err_y_m=zeros(stp_fin,1);
inclinazione_x=zeros(stp_fin,1);
inclinazione_y=zeros(stp_fin,1);

ctrl_x=zeros(stp_fin,1);
ctrl_y=zeros(stp_fin,1);
ctrl_z=zeros(stp_fin,1);
cum_err_z = zeros(stp_fin,1);
cum_err_x_m = zeros(stp_fin,1);
cum_err_y_m = zeros(stp_fin,1);
ctrl_head=zeros(stp_fin,1);
err_head=zeros(stp_fin,1);
cum_err_head=zeros(stp_fin,1);
inclinazione_y_set=zeros(stp_fin,1);
err_incl_y=zeros(stp_fin,1);
cum_err_incl_y=zeros(stp_fin,1);
inclinazione_x_set=zeros(stp_fin,1);
err_incl_x=zeros(stp_fin,1);
cum_err_incl_x=zeros(stp_fin,1);
thrust_set=zeros(stp_fin,4);
thrust_hover = zeros(stp_fin,1);

%Simulazione
acc=zeros(stp_fin,3);
vel=zeros(stp_fin,3);
pos=zeros(stp_fin,3);

v_mot=zeros(4,1);
J=zeros(4,1);
heading = zeros(stp_fin,1);
heading_set = zeros(stp_fin,1);
thrust=zeros(stp_fin,4);
thrust_rimanente=zeros(stp_fin,1);
power_drain=zeros(stp_fin,4);
momento_attrito=zeros(4,1);
alfa_local=zeros(stp_fin,3);
omega_local=zeros(stp_fin,3);
omega_motori=zeros(stp_fin,4);
quat=zeros(stp_fin,4);

%% Condizioni iniziali
% quat(1,:)=[0.908422885334012 -0.174593786583050 -0.376476547973490 0.0505002961772440];
% quat(1,:)=[cos(45*pi/180/2),0,0,sin(45*pi/180/2)];
quat(1,:)=[1,0,0,0];
% quat(1,:)=[cosd(70),1*sind(70),0,0];
pos(1,3) = 0;

rpm(1,:)=[3907,3907,3907,3907];
omega_motori(1,:)=rpm(1,:).*[1,-1,1,-1]*2*pi/60;

%% Proprietà velivolo
%di massa
massa = 1;

Ixx=0.03;
Iyy=0.03;
Izz=Ixx+Iyy;
I=[Ixx,0,0;0,Iyy,0;0,0,Izz];
I_inv=[1/Ixx,0,0;0,1/Iyy,0;0,0,1/Izz];

I_elica=1.5e-6;

%geometriche
dist=0.5;
pos_mot(:,1)=[dist;0;0];
pos_mot(:,2)=[0;dist;0];
pos_mot(:,3)=[-dist;0;0];
pos_mot(:,4)=[0;-dist;0];
diam_prop=0.25;
drag_coeff=[0.25;0.25;0.75];

%dinamiche
g=9.81;
thrust_max=8;
coeff_sicurezza=1;

incl_max=5;
distanza_raggiungimento_incl_max=1.5;
vel_salita_max=1.5;
vel_discesa_max=-1.5;

%% Parametri controllati dall'elettronica

x_set = [1];
y_set = [0,1,-1,0,0];
z_set = [-1,1,1,1,1];
heading_waypoint = [0,180,-45,-120,0].*pi/180;
waypoint = 1;
waypoint_max = min([ numel(x_set), numel(y_set), numel(z_set), numel(heading_waypoint) ]);
toll_waypoint=0.01;
rallentamento_ctrl = 10;
dt_ctrl = dt * rallentamento_ctrl;
incl_max = sind(15);

blocca_al_termine = false;

take_off = true;

%% Parametri PID espliciti
kp=0.1;
kd=0.2;
ki=0;

kpi=0.02;
kdi=0;
kii=0;

kpz=0.05;
kdz=0;
kiz=0;

kph=15;
kdh=20;
kih=0;

% kpr=15;
% kdr=0;
% kir=0;

%% Parametri PID impliciti
A=kp+ki*dt_ctrl/2+kd/dt_ctrl;
B=ki*dt_ctrl/2-kd/dt_ctrl;
C=ki;

Ai=kpi+kii*dt_ctrl/2+kdi/dt_ctrl;
Bi=kii*dt_ctrl/2-kdi/dt_ctrl;
Ci=kii;

Az=kpz+kiz*dt_ctrl/2+kdz/dt_ctrl;
Bz=kiz*dt_ctrl/2-kdz/dt_ctrl;
Cz=kiz;

Ah=kph+kih*dt_ctrl/2+kdh/dt_ctrl;
Bh=kih*dt_ctrl/2-kdh/dt_ctrl;
Ch=kih;

% Ar=kpr+kir*dt_ctrl/2+kdr/dt_ctrl;
% Br=kir*dt_ctrl/2-kdr/dt_ctrl;
% Cr=kir;

clear kp kd ki
clear kpz kdz kiz
clear kph kdh kih

%% Ciclo di calcolo
tic
for stp=2:stp_fin
    quat_conj=quat(stp-1,:).*[1,-1,-1,-1];
       
    %% %%%%%%%%%%%%%%%%%%% Controllo del volo %%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    if rem(stp , 10)==0
        %% Calcolo l'errore lungo l'asse x e y delle terna mobile e verifico il valore della componente dei versori x e y rispetto all'asse z fisso
        err_x(stp)=x_set(waypoint)-pos(stp-1,1);
        err_y(stp)=y_set(waypoint)-pos(stp-1,2);
        err_z(stp)=z_set(waypoint)-pos(stp-1,3);
        heading_set(stp) = heading_waypoint(waypoint); % angolo tra la direzione x locale e x fissa desiderato (in gradi)

        if ( (err_x(stp)^2 + err_y(stp)^2 + err_z(stp)^2)<toll_waypoint )
            if (waypoint<waypoint_max)
                waypoint = waypoint+1;
            elseif blocca_al_termine == true
                stp_fin = stp;
                break;
            end
        end

        U=quatmultiply(quat(stp-1,:),quatmultiply([0 1 0 0],quat_conj)); %porto l'asse x mobile nel riferimento fisso
        inclinazione_x(stp)=-U(4);
        U_2d=U(2:3);
        U_2d=U_2d./norm(U_2d);
        err_x_m(stp)=U_2d*[err_x(stp);err_y(stp)];
    %     err_y_m(stp)=real(sqrt(err_x(stp)^2+err_y(stp)^2-err_x_m(stp)^2));

        V=quatmultiply(quat(stp-1,:),quatmultiply([0 0 1 0],quat_conj));
        inclinazione_y(stp)=-V(4); %% se l'inclinazione_y è positiva ci si sposta a sinistra
        V_2d=V(2:3);
        V_2d=V_2d./norm(V_2d);
        err_y_m(stp)=V_2d*[err_x(stp);err_y(stp)];
        
        %% Mantenimento hovering
        thrust_hover(stp) = massa*g/(body_to_earth([0;0;1],quat(stp-1,:))'*[0;0;1])/4;
        if thrust_hover(stp)>thrust_max
            thrust_hover(stp) = thrust_max;
            disp('Spinta richiesta superiore alla spinta massima')
        end
        thrust_rimanente(stp) = min( (thrust_max - thrust_hover(stp)) , thrust_hover(stp));

        
        %% Correzione quota
        ctrl_z(stp)=Az*err_z(stp)+Bz*err_z(stp-1)+Cz*cum_err_z(stp-1);
        cum_err_z(stp)=cum_err_z(stp-1)+(err_z(stp)+err_z(stp-1))/2*dt_ctrl;
      
        thrust_rimanente(stp) = thrust_rimanente(stp) - abs(ctrl_z(stp));

        %% Inclinazione
        inclinazione_x_set(stp) = Ai*err_x_m(stp)+Bi*err_x_m(stp-1)+Ci*cum_err_x_m(stp-1);
        cum_err_incl_x(stp)=cum_err_x_m(stp-1)+(err_x_m(stp)+err_x_m(stp-1))/2*dt_ctrl;
        if inclinazione_x_set(stp)>incl_max
            inclinazione_x_set(stp) = incl_max;
        elseif inclinazione_x_set(stp)<-incl_max
            inclinazione_x_set(stp) = -incl_max;
        end
        
        err_incl_x(stp)=inclinazione_x_set(stp)-inclinazione_x(stp);
        ctrl_x(stp)=A*err_incl_x(stp)+B*err_incl_x(stp-1)+C*cum_err_incl_x(stp-1);
        cum_err_incl_x(stp)=cum_err_incl_x(stp-1)+(err_incl_x(stp)+err_incl_x(stp-1))/2*dt_ctrl;
        
        
        inclinazione_y_set(stp) = Ai*err_y_m(stp)+Bi*err_y_m(stp-1)+Ci*cum_err_y_m(stp-1);
        cum_err_incl_y(stp)=cum_err_y_m(stp-1)+(err_y_m(stp)+err_y_m(stp-1))/2*dt_ctrl;
        if inclinazione_y_set(stp)>incl_max
            inclinazione_y_set(stp) = incl_max;
        elseif inclinazione_y_set(stp)<-incl_max
            inclinazione_y_set(stp) = -incl_max;
        end

        err_incl_y(stp)=inclinazione_y_set(stp)-inclinazione_y(stp);
        ctrl_y(stp)=A*err_incl_y(stp)+B*err_incl_y(stp-1)+C*cum_err_incl_y(stp-1);
        cum_err_incl_y(stp)=cum_err_incl_y(stp-1)+(err_incl_y(stp)+err_incl_y(stp-1))/2*dt_ctrl;
        
        
        
        
        thrust_rimanente(stp) = thrust_rimanente(stp) - max (abs(ctrl_x(stp)),abs(ctrl_y(stp)));


        %% Controllo rotazione attorno all'asse Z
        heading(stp) = atan2(U_2d(2),U_2d(1));

        scelta(1) = heading_set(stp) - ( 2*pi + heading(stp));
        scelta(2) = heading_set(stp) - heading(stp);
        scelta(3) = (2*pi + heading_set(stp)) - ( 2*pi + heading(stp));
        scelta(4) = (2*pi + heading_set(stp)) - heading(stp);
        [~,indice] = min (abs(scelta) );

        err_head(stp) = scelta(indice);
        ctrl_head(stp)=Ah*err_head(stp)+Bh*err_head(stp-1)+Ch*cum_err_head(stp-1);
        cum_err_head(stp)=cum_err_head(stp-1)+(err_head(stp)+err_head(stp-1))/2*dt_ctrl;

    %     err_vel_head = ctrl_vel_head - omega_local(stp,3);
    %     ctrl_head(stp)=Ahv*err_vel_head(stp)+Bhv*err_vel_head(stp-1)+Chv*cum_err_vel_head(stp-1);
    %     cum_err_vel_head(stp)=cum_err_vel_head(stp-1)+(err_vel_head(stp)+err_vel_head(stp-1))/2*dt_ctrl;

        if ctrl_head(stp)>thrust_rimanente(stp)
            ctrl_head(stp)=thrust_rimanente(stp);
        elseif ctrl_head(stp)<-thrust_rimanente(stp)
            ctrl_head(stp)=-thrust_rimanente(stp);
        end

        ctrl_head(stp)=0; %% BLOCCO CONTROLLO HEADING ATTIVO
        
        %% Imposto velocità rotazione    
        thrust_set(stp,1) = min( max( thrust_hover(stp) + ctrl_z(stp) - ctrl_x(stp) - ctrl_head(stp) ,0) , thrust_max);
        thrust_set(stp,3) = min( max( thrust_hover(stp) + ctrl_z(stp) + ctrl_x(stp) - ctrl_head(stp) ,0) , thrust_max);

        thrust_set(stp,2) = min( max( thrust_hover(stp) + ctrl_z(stp) - ctrl_y(stp) + ctrl_head(stp) ,0) , thrust_max);
        thrust_set(stp,4) = min( max( thrust_hover(stp) + ctrl_z(stp) + ctrl_y(stp) + ctrl_head(stp) ,0) , thrust_max);
        
        if thrust_rimanente(stp)<0 
            disp('Il controllo indurrà una rotazione su yaw indesiderata')
        end
        
        
    else
        thrust_set(stp,:) = thrust_set(stp-1,:);
        
        inclinazione_y(stp) = inclinazione_y(stp-1);
        err_incl_y(stp) = err_incl_y(stp-1);
        cum_err_incl_y(stp) = cum_err_incl_y(stp-1);
        inclinazione_x(stp) = inclinazione_x(stp-1);
        err_incl_x(stp) = err_incl_x(stp-1);
        cum_err_incl_x(stp) = cum_err_incl_x(stp-1);
        cum_err_z(stp) = cum_err_z(stp-1);
        err_head(stp) = err_head(stp-1);
        cum_err_head(stp) = cum_err_head(stp-1);
%         err_vel_head(stp) = err_vel_head(stp-1);
%         cum_err_vel_head(stp) = cum_err_vel_head(stp-1);
        inclinazione_y_set(stp) = inclinazione_y_set(stp-1);
        inclinazione_x_set(stp) = inclinazione_x_set(stp-1);
    end
    
        
    
    %% %%%%%%%%%%%%%%%%%%%% Dinamica del velivolo %%%%%%%%%%%%%%%%%%%%%%%%
    
    %% Calcolo momento d'attrito
    accelerazione_motore = zeros(4,1);
    for mot=1:4 %per ogni motore calcolo, in funzione della spinta (thrust), gli rpm e il momento d'attrito
        
        v_mot(mot)= earth_to_body( [0;0;1] , quat(stp-1,:)' )' * ( vel(stp-1,:)' + cross( omega_local(stp,:)', pos_mot(:,mot) ) );
%         v_mot(mot)= earth_to_body( [0;0;1] , quat(stp-1,:)' )' * vel(stp-1,:)';

        if v_mot(mot)<0 
            v_mot(mot)=0;
        end
        
        if rpm(stp-1,mot)==0
            J(mot)=J(mot);
        else
            J(mot)=60/rpm(stp-1,mot)*v_mot(mot)/diam_prop;  % vedi http://www.drivecalc.de/PropCalc/PCHelp/Help.html#general
        end
        
        scarto_thrust = thrust_set(stp,mot)-thrust(stp-1,mot);
        if scarto_thrust>2
            accelerazione_motore(mot) = 10000;
        elseif scarto_thrust<-2
            accelerazione_motore(mot) = -10000;
        else
                accelerazione_motore(mot) = scarto_thrust *10000/2;                
        end
        
        rpm(stp,mot) = rpm(stp-1,mot) + accelerazione_motore(mot)*dt;
        
        [ thrust(stp,mot), momento_attrito(mot)] = setTtrue(rpm(stp,mot),J(mot));
        if thrust(stp,mot) > thrust_max
            disp('Warning: potenza massima raggiunta');
        end
    end
    omega_motori(stp,:)=rpm(stp,:).*[1,-1,1,-1]*2*pi/60;
    mom_attrito_tot=sum(momento_attrito.*[-1;+1;-1;+1]);   %sarà lungo l'asse z della terna solidale al velivolo

    %% Calcolo variazione modulo della velocità angolare dei motori
    if omega_motori(stp,:)~=omega_motori(stp-1,:)
        acc_angolare_motori=(omega_motori(stp,:)-omega_motori(stp-1,:))/dt;
        mom_accelerazione_motori=-acc_angolare_motori.*I_elica;    %nel riferimento relativo
    else
        mom_accelerazione_motori=0;
    end
    mom_accelerazione_motori=sum(mom_accelerazione_motori);

    %% Calcolo variazione in direzione della velocità angolare dei motori
    mom_giroscopico_motori=zeros(3,4);

    for mot=1:4
        if (mot==2)||(mot==4)
            verso_momento=-1;
        else
            verso_momento=1;
        end

        if norm(cross(omega_local(stp-1,:),[0,0,verso_momento]))>0
            mom_giroscopico_motori(:,mot)=cross(omega_local(stp-1,:),[0,0,verso_momento])./norm(cross(omega_local(stp-1,:),[0,0,verso_momento]))*I_elica*norm(omega_motori(stp,mot));
        end
    end
    mom_giroscopico_motori=sum(mom_giroscopico_motori,2);

    %% Calcolo del momento dovuto alle forze di sollevamento dei motori
    mom_sollevamento=[(thrust(stp,2)-thrust(stp,4))*dist;(-thrust(stp,1)+thrust(stp,3))*dist;0];

    %% Calcolo del momento complessivo (rispetto al riferimento mobile)
    M_local=[0;0;mom_attrito_tot+mom_accelerazione_motori]+mom_giroscopico_motori+mom_sollevamento;   %nel riferimento MOBILE

    %% SOVRASCRITTURA MOMENTI E FORZE
    % M_local=[0;0;0];   %nel riferimento MOBILE
    % thrust=[0,0,0,0];

    %% Riporto forze e momenti nel rif.assoluto
    F_local=[0;0;sum(thrust(stp,:))];
    F=quatmultiply(quat(stp-1,:),quatmultiply([0,F_local.'],quat_conj));

    F=F.';
    F=F(2:4,1)+[0;0;-massa*g]-drag_coeff.*vel(stp-1,:).';

    %% Calcolo degli spostamenti
    acc(stp,:)=F.'/massa;    
    vel(stp,:)=vel(stp-1,:)+acc(stp,:)*dt;
    pos(stp,:)=pos(stp-1,:)+vel(stp,:)*dt;
    
    if (take_off == false)&&(pos(stp,3)<0)
        pos(stp,3) = 0;
%         if vel(stp,3)<0
%             vel(stp,3) = 0;
%         end
    else
        take_off = true;
    end
    
    %% Calcolo dell'orientamento
    alfa_local(stp,:)=I_inv*(M_local-(cross(omega_local(stp-1,:),I*omega_local(stp-1,:).')).');
    omega_local(stp,:)=omega_local(stp-1,:)+alfa_local(stp,:)*dt;

    if norm(omega_local(stp,:))~=0
        quat(stp,:)=quatmultiply(quat(stp-1,:),[cos(0.5*norm(omega_local(stp,:))*dt),omega_local(stp,:)/norm(omega_local(stp,:))*sin(0.5*norm(omega_local(stp,:))*dt)]);
    else
        quat(stp,:)=quat(stp-1,:);
    end
    
%     %% Salvataggio dati per test
%     acc_sens=quatmultiply(quat_conj,quatmultiply([0,(acc(stp,:)+[0,0,-g])],quat(stp-1,:)));
%     acc_sensore(:,stp)=acc_sens(2:4)';
%     omega_sensore(:,stp)=omega_local(stp,:)';
    
end
toc

%% Stima durata batterie
cap_batterie=5000;
eta_m=0.6;
lavoro=sum(sum(power_drain))*dt/eta_m;
minuti_volo=cap_batterie/1000*11.1*3600/lavoro/60*stp_fin*dt;

%% Grafici di volo
close all
scrsz = get(0,'ScreenSize');

figure('OuterPosition',[25 45 scrsz(3)/2-12 scrsz(4)/2],'Name','Accelerazione','NumberTitle','off')
hold on
plot((1:stp_fin)*dt,acc(1:stp_fin,1),'b');
plot((1:stp_fin)*dt,acc(1:stp_fin,2),'Color',[0 0.5 0.2]);
legend('AccX','AccY');

figure('OuterPosition',[25 45 scrsz(3)/2-12 scrsz(4)/2],'Name','Inclinazione','NumberTitle','off')
hold on
plot((1:stp_fin)*dt,asind(inclinazione_x(1:stp_fin)),'b');
plot((1:stp_fin)*dt,asind(inclinazione_y(1:stp_fin)),'Color',[0 0.5 0.2]);
plot((1:stp_fin)*dt,asind(inclinazione_x_set(1:stp_fin)),'r');
plot((1:stp_fin)*dt,asind(inclinazione_y_set(1:stp_fin)),'c');
legend('InclX','InclY','InclXset','InclYset');

figure('OuterPosition',[scrsz(3)/2+12 scrsz(4)/2 scrsz(3)/2-12 scrsz(4)/2],'Name','Spinta','NumberTitle','off')
hold on
plot((1:stp_fin)*dt,thrust(1:stp_fin,1),'b');
plot((1:stp_fin)*dt,thrust(1:stp_fin,1),'Color',[0 0.5 0.2]);
plot((1:stp_fin)*dt,thrust(1:stp_fin,1),'r');
plot((1:stp_fin)*dt,thrust(1:stp_fin,1),'c');
plot((1:stp_fin)*dt,thrust_set(1:stp_fin,1),'g');
plot((1:stp_fin)*dt,thrust_set(1:stp_fin,1),'m');
plot((1:stp_fin)*dt,thrust_set(1:stp_fin,1),'Color',[0 0.2 0.5]);
plot((1:stp_fin)*dt,thrust_set(1:stp_fin,1),'k');
legend('spinta1','spinta2','spinta3','spinta4','s1set','s2set','s3set','s4set')

figure('OuterPosition',[scrsz(3)/2+12 scrsz(4)/2 scrsz(3)/2-12 scrsz(4)/2],'Name','Velocità','NumberTitle','off')
hold on
plot((1:stp_fin)*dt,vel(1:stp_fin,1)*3.6,'b');
plot((1:stp_fin)*dt,vel(1:stp_fin,2)*3.6,'Color',[0 0.5 0.2]);
plot((1:stp_fin)*dt,vel(1:stp_fin,3)*3.6,'r');
legend('velX','velY','velZ')

figure('OuterPosition',[25 scrsz(4)/2 scrsz(3)/2-12 scrsz(4)/2],'Name','Posizione','NumberTitle','off')
hold on
plot((1:stp_fin)*dt,pos(1:stp_fin,1),'b');
plot((1:stp_fin)*dt,pos(1:stp_fin,2),'Color',[0 0.5 0.2]);
plot((1:stp_fin)*dt,pos(1:stp_fin,3),'r');
legend('posX','posY','Quota')




% figure('OuterPosition',[25 scrsz(4)/2 scrsz(3)/2-12 scrsz(4)/2],'Name','Heading','NumberTitle','off')
% hold on
% plot((1:stp_fin)*dt,heading(1:stp_fin),'b');
% plot((1:stp_fin)*dt,heading_set(1:stp_fin),'Color',[0 0.5 0.2]);
% plot((1:stp_fin)*dt,err_head(1:stp_fin),'r');
% ylim([-3.5 3.5])
% legend('heading','heading set','err head')
% 
% figure('OuterPosition',[25 45 scrsz(3)/2-12 scrsz(4)/2],'Name','ControlloHead','NumberTitle','off')
% hold on
% plot((1:stp_fin)*dt,ctrl_head(1:stp_fin),'r');
% legend('controllo heading');



figure('OuterPosition',[scrsz(3)/2+12 45 scrsz(3)/2-12 scrsz(4)/2],'Name','RPM','NumberTitle','off')
hold on
plot((1:stp_fin)*dt,rpm(1:stp_fin,1),'c');
plot((1:stp_fin)*dt,rpm(1:stp_fin,2),'g');
plot((1:stp_fin)*dt,rpm(1:stp_fin,3),'m');
plot((1:stp_fin)*dt,rpm(1:stp_fin,4),'k');
legend('rpm1','rpm2','rpm3','rpm4');

%% Disegno velivolo
% drawCube2(pos(1:stp_fin,:)',[1;1;0.2],quat(1:stp_fin,:)','blue',dt);
