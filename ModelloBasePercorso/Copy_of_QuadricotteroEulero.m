close all
clear all
clc

raggiunto=0;

stp=2;
dt=0.001;
stp_fin=10000;

err_x=zeros(stp_fin,1);
err_y=zeros(stp_fin,1);
err_z=zeros(stp_fin,1);
err_x_m=zeros(stp_fin,1);
err_y_m=zeros(stp_fin,1);
ctrl_x=zeros(stp_fin,1);
ctrl_y=zeros(stp_fin,1);
ctrl_z=zeros(stp_fin,1);
ctrl_head=zeros(stp_fin,1);
vel_x_set=zeros(stp_fin,1);
vel_y_set=zeros(stp_fin,1);
vel_z_set=zeros(stp_fin,1);
err_vel_x=zeros(stp_fin,1);
err_vel_y=zeros(stp_fin,1);
err_vel_z=zeros(stp_fin,1);
err_head=zeros(stp_fin,1);
cum_err_vel_x=zeros(stp_fin,1);
cum_err_vel_y=zeros(stp_fin,1);
cum_err_vel_z=zeros(stp_fin,1);
cum_err_head=zeros(stp_fin,1);
cum_err_vel_head=zeros(stp_fin,1);
cum_err_acc_head=zeros(stp_fin,1);
acc_x_set=zeros(stp_fin,1);
cum_err_acc_x=zeros(stp_fin,1);
err_acc_x=zeros(stp_fin,1);
acc_y_set=zeros(stp_fin,1);
cum_err_acc_y=zeros(stp_fin,1);
err_acc_y=zeros(stp_fin,1);
acc=zeros(stp_fin,3);
vel=zeros(stp_fin,3);
pos=zeros(stp_fin,3);
inclinazione_x=zeros(stp_fin,1);
inclinazione_y=zeros(stp_fin,1);
v_mot=zeros(4,1);
J=zeros(4,1);
thrust=zeros(stp_fin,4);
thrust_rimanente=zeros(stp_fin,1);
power_drain=zeros(stp_fin,4);
momento_attrito=zeros(4,1);
alfa_local=zeros(stp_fin,3);
omega_local=zeros(stp_fin,3);
omega_motori=zeros(stp_fin,4);
quat=zeros(stp_fin,4);

acc_sensore=zeros(3,stp_fin);
acc_sensore(:,1)=[0;0;-9.8];
omega_sensore=zeros(3,stp_fin);

%% Parametri elica
%              |rpm    |Thrust (N)| Power_drain (W)|
tabella_elica= [0,      0,          0;
                2000,	0.171,      0.6;
                4000,	0.751,      4.3;
                6000,	1.810,      14.3;
                8000,	3.387,      34.3;
                10000,	5.421,      67.5;
                12000,	7.889,      117.2;
                14000,	10.832,     187];
                
%% Condizioni iniziali
% quat(1,:)=[0.908422885334012 -0.174593786583050 -0.376476547973490 0.0505002961772440];
% quat(1,:)=[cos(45*pi/180/2),0,0,sin(45*pi/180/2)];
quat(1,:)=[1,0,0,0];
% quat(1,:)=[cosd(70),1*sind(70),0,0];

rpm(1,:)=[6896.5,6896.5,6896.5,6896.5];
omega_motori(1,:)=rpm(1,:).*[1,-1,1,-1]*2*pi/60;

%% Proprietà velivolo
%di massa
m=1;

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
drag_coeff=[0.25;0.25;0.25];

%dinamiche
g=9.8;
thrust_hover=g/4;
thrust_max=8;
coeff_sicurezza=1;

vel_laterale_max=5;
distanza_raggiungimento_vel=5;
vel_salita_max=2.5;
vel_discesa_max=-1.5;

%% Parametri controllati dall'elettronica
x_set=5;
y_set=3;
z_set=5;

%% Parametri PID espliciti
kpa=4;
kda=0;
kia=0;

kp=7.5;
kd=9;
ki=0;

kpz=5;
kdz=1.2;
kiz=0;

kph=15;
kdh=20;
kih=0;

%% Parametri PID impliciti
A=kp+ki*dt/2+kd/dt;
B=ki*dt/2-kd/dt;
C=ki;

Aa=kpa+kia*dt/2+kda/dt;
Ba=kia*dt/2-kda/dt;
Ca=kia;

Az=kpz+kiz*dt/2+kdz/dt;
Bz=kiz*dt/2-kdz/dt;
Cz=kiz;

Ah=kph+kih*dt/2+kdh/dt;
Bh=kih*dt/2-kdh/dt;
Ch=kih;

%% Ciclo di calcolo
tic
for stp=2:stp_fin
    quat_conj=quat(stp-1,:).*[1,-1,-1,-1];
       
    %% %%%%%%%%%%%%%%%%%%% Controllo del volo %%%%%%%%%%%%%%%%%%%%%%%%%%%%   

      
    %% Calcolo l'errore lungo l'asse x e y delle terna mobile e verifico il valore della componente dei versori x e y rispetto all'asse z fisso
    err_x(stp)=x_set-pos(stp-1,1);
    err_y(stp)=y_set-pos(stp-1,2);
    err_z(stp)=z_set-pos(stp-1,3);
    
    U=quatmultiply(quat(stp-1,:),quatmultiply([0 100 0 0],quat_conj)); %porto l'asse x mobile nel riferimento fisso
    inclinazione_x(stp)=-U(4);
    U_2d=U(2:3);
    U_2d=U_2d./norm(U_2d);
    err_x_m(stp)=U_2d*[err_x(stp);err_y(stp)];
%     err_y_m(stp)=real(sqrt(err_x(stp)^2+err_y(stp)^2-err_x_m(stp)^2));

    V=quatmultiply(quat(stp-1,:),quatmultiply([0 0 100 0],quat_conj));
    inclinazione_y(stp)=-V(4);
    V_2d=V(2:3);
    V_2d=V_2d./norm(V_2d);
    err_y_m(stp)=V_2d*[err_x(stp);err_y(stp)];
    
    if err_x_m(stp)>distanza_raggiungimento_vel
        vel_x_set(stp)=vel_laterale_max;
    elseif err_x_m(stp)<-distanza_raggiungimento_vel
        vel_x_set(stp)=-vel_laterale_max;
    else
        vel_x_set(stp)=err_x_m(stp)*vel_laterale_max/distanza_raggiungimento_vel;
    end
    
    if err_y_m(stp)>distanza_raggiungimento_vel
        vel_y_set(stp)=vel_laterale_max;
    elseif err_y_m(stp)<-distanza_raggiungimento_vel
        vel_y_set(stp)=-vel_laterale_max;
    else
        vel_y_set(stp)=err_y_m(stp)*vel_laterale_max/distanza_raggiungimento_vel;
    end
    
    if err_z(stp)>2.5
        vel_z_set(stp)=vel_salita_max;
    elseif err_z(stp)<-2.5
        vel_z_set(stp)=vel_discesa_max;
    else
        if err_z(stp)>=0
            vel_z_set(stp)=err_z(stp)*vel_salita_max/2.5;
        else
            vel_z_set(stp)=err_z(stp)*-vel_discesa_max/2.5;
        end
    end
        
    %% Calcolo e correggo la quota
    err_vel_z(stp)=vel_z_set(stp)-vel(stp-1,3);
    ctrl_z(stp)=Az*err_vel_z(stp)+Bz*err_vel_z(stp-1)+Cz*cum_err_vel_z(stp-1);
    cum_err_vel_z(stp)=cum_err_vel_z(stp-1)+(err_vel_z(stp)+err_vel_z(stp-1))/2*dt;

    if ctrl_z(stp)>(100-thrust_hover*100/thrust_max)
        ctrl_z(stp)=100-thrust_hover*100/thrust_max;
        disp('Warning: potenza massima raggiunta');
    elseif ctrl_z(stp)<-thrust_hover*100/thrust_max
        ctrl_z(stp)=-thrust_hover*100/thrust_max;
    end
    
    % thrust da 0 a 100
    thrust_rimanente(stp)=min( (100-(ctrl_z(stp)+thrust_hover*100/thrust_max))*coeff_sicurezza , (ctrl_z(stp)+thrust_hover*100/thrust_max) );
    % se per rimanere in quota devo assorbire 67 su 100 di thrust (esempio) il
    % thrust rimanente è 100-67, ma se per rimanere in quota ho bisogno di
    % 35 il trust rimanente è di 35!!!
    
    %% Calcolo forza necessaria per raggiungere la velocità desiderata
    err_vel_x(stp)=vel_x_set(stp)-vel(stp-1,:)*[U_2d.';0];
    acc_x_set(stp)=Aa*err_vel_x(stp)+Ba*err_vel_x(stp-1)+Ca*cum_err_vel_x(stp-1);
    cum_err_vel_x(stp)=cum_err_vel_x(stp-1)+(err_vel_x(stp)+err_vel_x(stp-1))/2*dt;
    
    if acc_x_set(stp)>3
        acc_x_set(stp)=3;
    elseif acc_x_set(stp)<-3
        acc_x_set(stp)=-3;
    end
    
    err_acc_x(stp)=acc_x_set(stp)-acc(stp-1,:)*[U_2d.';0];
    ctrl_x(stp)=A*err_acc_x(stp)+B*err_acc_x(stp-1)+C*cum_err_acc_x(stp-1);
    cum_err_acc_x(stp)=cum_err_acc_x(stp-1)+(err_acc_x(stp)+err_acc_x(stp-1))/2*dt;
    
    if ctrl_x(stp)>thrust_rimanente(stp)
        ctrl_x(stp)=thrust_rimanente(stp);
    elseif ctrl_x(stp)<-thrust_rimanente(stp)
        ctrl_x(stp)=-thrust_rimanente(stp);
    end
    
    thrust_rimanente(stp) = thrust_rimanente(stp) - abs(ctrl_x(stp));
    
    err_vel_y(stp)=vel_y_set(stp)-vel(stp-1,:)*[V_2d.';0];
    acc_y_set(stp)=Aa*err_vel_y(stp)+Ba*err_vel_y(stp-1)+Ca*cum_err_vel_y(stp-1);
    cum_err_vel_y(stp)=cum_err_vel_y(stp-1)+(err_vel_y(stp)+err_vel_y(stp-1))/2*dt;
    
    if acc_y_set(stp)>3
        acc_y_set(stp)=3;
    elseif acc_y_set(stp)<-3
        acc_y_set(stp)=-3;
    end
    
    err_acc_y(stp)=acc_y_set(stp)-acc(stp-1,:)*[V_2d.';0];    
    ctrl_y(stp)=A*err_acc_y(stp)+B*err_acc_y(stp-1)+C*cum_err_acc_y(stp-1);
    cum_err_acc_y(stp)=cum_err_acc_y(stp-1)+(err_acc_y(stp)+err_acc_y(stp-1))/2*dt;
    
    if ctrl_y(stp)>thrust_rimanente(stp)
        ctrl_y(stp)=thrust_rimanente(stp);
    elseif ctrl_y(stp)<-thrust_rimanente(stp)
        ctrl_y(stp)=-thrust_rimanente(stp);
    end
    
    thrust_rimanente(stp) = thrust_rimanente(stp) - abs(ctrl_y(stp));
    
    %% Controllo rotazione attorno all'asse Z
    
    heading_set = 180*pi/180; % angolo tra la direzione x locale e x fissa desiderato (in gradi)
    heading(stp) = atan2(U_2d(2),U_2d(1));
    
    scelta(1) = heading_set - ( 2*pi + heading(stp));
    scelta(2) = heading_set - heading(stp);
    scelta(3) = (2*pi + heading_set) - ( 2*pi + heading(stp));
    scelta(4) = (2*pi + heading_set) - heading(stp);
    [~,indice] = min (abs(scelta));
    
    err_head(stp) = scelta(indice);
 
    ctrl_head(stp)=Ah*err_head(stp)+Bh*err_head(stp-1)+Ch*cum_err_head(stp-1);
    cum_err_head(stp)=cum_err_head(stp-1)+(err_head(stp)+err_head(stp-1))/2*dt;
    
    if ctrl_head(stp)>thrust_rimanente(stp)
        ctrl_head(stp)=thrust_rimanente(stp);
    elseif ctrl_head(stp)<-thrust_rimanente(stp)
        ctrl_head(stp)=-thrust_rimanente(stp);
    end
    
    thrust_rimanente(stp) = thrust_rimanente(stp) - abs(ctrl_head(stp));
    
    %% Imposto velocità rotazione
    thrust(stp,1)=(thrust_hover*100/thrust_max+ctrl_z(stp)-ctrl_x(stp)-ctrl_head(stp))/100*thrust_max;
    thrust(stp,3)=(thrust_hover*100/thrust_max+ctrl_z(stp)+ctrl_x(stp)-ctrl_head(stp))/100*thrust_max;

    thrust(stp,2)=(thrust_hover*100/thrust_max+ctrl_z(stp)-ctrl_y(stp)+ctrl_head(stp))/100*thrust_max;
    thrust(stp,4)=(thrust_hover*100/thrust_max+ctrl_z(stp)+ctrl_y(stp)+ctrl_head(stp))/100*thrust_max;
    
    rpm(stp,:)=interp1(tabella_elica(:,2),tabella_elica(:,1),thrust(stp,:),'pchip');
    power_drain(stp,:)=interp1(tabella_elica(:,2),tabella_elica(:,3),thrust(stp,:),'pchip');
    omega_motori(stp,:)=rpm(stp,:).*[1,-1,1,-1]*2*pi/60;
    
    %% %%%%%%%%%%%%%%%%%%%% Dinamica del velivolo %%%%%%%%%%%%%%%%%%
    
    %% Calcolo momento d'attrito
    for mot=1:4 %per ogni motore calcolo, in funzione di rpm, la spinta e il momento d'attrito

    %     v_mot(mot)=(rot_cardano*[0;0;1]).'*(P_1+cross(omega_local,pos_mot(:,mot))); %DA VERIFICARE proietto la velocità del punto in cui è fissata l'elica sull'asse di rotazione per calcolare la velocità che "vede la ventola"
        %se dovesse accadere che una ventola, nonostante stia spingendo, va 
        %"all'indietro" faccio in modo che la velocità con cui viene calcolato
        %il coefficiente di avanzamento J sia al più pari a 0 e non negativo
        %cioè è come se la pala fosse ferma. PER ORA J NON È UTILIZZATO
    
%         
%     if v_mot(mot)<0 
%         v_mot(mot)=0;
%     end
    
%     J(mot)=60/rpm(mot)*v_mot(mot)/diam_prop;  %vedi http://www.drivecalc.de/PropCalc/PCHelp/Help.html#general
    
    
    
%     [rpm(stp,mot),momento_attrito(mot),thrust(stp,mot)] = setRPM(J,thrust(stp,mot));
    
    
        if rpm(stp,mot)==0
            momento_attrito(mot)=0;
        else
            momento_attrito(mot)=power_drain(stp,mot)/-omega_motori(stp,mot);  %per tenere conto che sono controrotanti
        end
    end
    
    mom_attrito_tot=sum(momento_attrito);   %sarà lungo l'asse z della terna solidale al velivolo

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
    F=F(2:4,1)+[0;0;-m*g]-drag_coeff.*vel(stp-1,:).';

    %% Calcolo degli spostamenti
    acc(stp,:)=F.'/m;    
    vel(stp,:)=vel(stp-1,:)+acc(stp,:)*dt;
    pos(stp,:)=pos(stp-1,:)+vel(stp,:)*dt;
    
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
scrsz = get(0,'ScreenSize');
figure('OuterPosition',[scrsz(3)/2+12 45 scrsz(3)/2-12 scrsz(4)/2],'Name','Inclinazione','NumberTitle','off')
hold on
plot((1:stp_fin)*dt,asind(inclinazione_x/100),'b');
plot((1:stp_fin)*dt,asind(inclinazione_y/100),'Color',[0 0.5 0.2]);
legend('InclX','InclY');

figure('OuterPosition',[25 45 scrsz(3)/2-12 scrsz(4)/2],'Name','Accelerazione','NumberTitle','off')
hold on
plot((1:stp_fin)*dt,acc(:,1),'b');
plot((1:stp_fin)*dt,acc(:,2),'Color',[0 0.5 0.2]);
plot((1:stp_fin)*dt,acc_x_set,'c');
plot((1:stp_fin)*dt,acc_y_set,'g');
legend('AccX','AccY');

figure('OuterPosition',[25 scrsz(4)/2 scrsz(3)/2-12 scrsz(4)/2],'Name','Posizione','NumberTitle','off')
hold on
plot((1:stp_fin)*dt,pos(:,1),'b');
plot((1:stp_fin)*dt,pos(:,2),'Color',[0 0.5 0.2]);
plot((1:stp_fin)*dt,pos(:,3),'r');
legend('posX','posY','Quota')

figure('OuterPosition',[scrsz(3)/2+12 scrsz(4)/2 scrsz(3)/2-12 scrsz(4)/2],'Name','Velocità','NumberTitle','off')
hold on
plot((1:stp_fin)*dt,vel(:,1),'b');
plot((1:stp_fin)*dt,vel(:,2),'Color',[0 0.5 0.2]);
plot((1:stp_fin)*dt,vel(:,3),'r');
plot((1:stp_fin)*dt,vel_x_set,'c');
plot((1:stp_fin)*dt,vel_y_set,'g');
plot((1:stp_fin)*dt,vel_z_set,'m');
legend('velX','velY','velZ')

%% Grafici rotazione Z
figure('OuterPosition',[25 scrsz(4)/2 scrsz(3)/2-12 scrsz(4)/2],'Name','RotazioneZ','NumberTitle','off')
hold on
plot((1:stp_fin)*dt,heading,'b');
plot((1:stp_fin)*dt,ones(stp_fin,1)*heading_set,'Color',[0 0.5 0.2]);
ylim([-pi pi])
legend('heading','heading set')

figure('OuterPosition',[25 45 scrsz(3)/2-12 scrsz(4)/2],'Name','Alfa','NumberTitle','off')
hold on
plot((1:stp_fin)*dt,ctrl_head,'r');
legend('controllo heading');

%% Disegno velivolo
drawCube2(pos',[1;1;0.2],quat','blue',dt);

%% Salvo misure per test
% save('C:\Users\Dario\Dropbox\Robotica\Quadricottero\ModelloSRCDKF\Misure.mat','acc_sensore','omega_sensore','pos','vel','quat')
% save('C:\Users\Dario\Dropbox\Robotica\Quadricottero\ModelloUKF\Misure.mat','acc_sensore','omega_sensore','pos','vel','quat')
