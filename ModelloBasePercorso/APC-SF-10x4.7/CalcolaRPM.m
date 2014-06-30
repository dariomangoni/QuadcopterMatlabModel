close all
clear all
clc

load 'C:\Users\Dario\Dropbox\Robotica\Quadricottero\ModelloBasePercorso\APC-SF-10x4.7\matrice_coeffMETRICmod.mat'

range_J = 0:0.025:0.65;
range_T = 0:0.1:10;
specchio_riassuntivo = zeros(numel(range_T)+1,numel(range_J)+1);
specchio_riassuntivo(2:end,1) = range_T; % al variare delle righe cambia T
specchio_riassuntivo(1,2:end) = range_J; % al variare delle colonne cambia J

specchio_rpm = zeros(numel(range_T)+1,numel(range_J)+1);
specchio_rpm(2:end,1) = range_T; % al variare delle righe cambia T
specchio_rpm(1,2:end) = range_J; % al variare delle colonne cambia J

specchio_Q = zeros(numel(range_T)+1,numel(range_J)+1);
specchio_Q(2:end,1) = range_T; % al variare delle righe cambia T
specchio_Q(1,2:end) = range_J; % al variare delle colonne cambia J

grado_x = size(matrice_coefficientiRPM,1)-1;
grado_y = size(matrice_coefficientiRPM,2)-1;
coeff=zeros(grado_x+1,1);

grado_xQ = size(matrice_coefficientiQ,1)-1;
grado_yQ = size(matrice_coefficientiQ,2)-1;

stpJ=2;

for J=range_J
    stpT=2;
    for T=range_T
        termine_noto = -T;
        for g=0:grado_x
            k=0;
            coeff(g+1)=0;
            while ((k+g)<=grado_y)
                coeff(g+1) = coeff(g+1)+matrice_coefficientiRPM(g+1,k+1)*J^k;
                k=k+1;
            end
        end

        coeff(1) = coeff(1)+termine_noto;
        rpm = poly_roots( coeff );
        rpm = [real(rpm),imag(rpm)];
        rpm_formatted = rpm;

        for k=1:grado_x
            if abs(rpm(k,2))>0.1
                rpm_formatted(k,1) = NaN;
            end
            if (rpm(k,1)<0) || (rpm(k,1)>9000)
                rpm_formatted(k,1) = NaN;
            end
        end

        flag_error = 0;
        for k=1:grado_x
            if ~isnan(rpm_formatted(k,1))
                rpm_set = rpm_formatted(k,1);
                flag_error=flag_error+1;
                specchio_rpm(stpT,stpJ) = rpm_set;
                
                % Calcolo Q corrispondente a rpm_set
                Q = 0;
                for riga=0:grado_yQ
                    for col=0:grado_xQ
                        Q = Q + matrice_coefficientiQ(riga+1,col+1)*rpm_set^riga*J^col;
                    end
                end
                specchio_Q(stpT,stpJ) = Q;
            end
        end
        
        if flag_error~=1
            disp(['Errore per J=',num2str(J),' e T=',num2str(T),' con ',num2str(flag_error),' velocità trovate']);
            specchio_rpm(stpT,stpJ) = NaN;
            specchio_Q(stpT,stpJ) = NaN;
        end
        
        specchio_riassuntivo(stpT,stpJ) = flag_error;
        
        stpT=stpT+1;
    end
    stpJ = stpJ+1;
end

% Calcola massima spinta al variare del coefficiente di avanzamento
for k=2:(stpJ-1)
    [max_rpm,indice] = max(specchio_rpm(2:end,k));
    if specchio_rpm(indice,1)>0
        max_thrust(k-1,1) = specchio_rpm(1,k);
        max_thrust(k-1,2) = specchio_rpm(indice,1);
        max_thrust(k-1,3) = specchio_rpm(indice,k);
    end
end

scrsz = get(0,'ScreenSize');
figure('OuterPosition',[scrsz(3)/2+12 scrsz(4)/2 scrsz(3)/2-12 scrsz(4)/2],'Name','Grafico Q','NumberTitle','off')
surf(range_J,range_T,specchio_Q(2:end,2:end));
xlabel('J: advance ratio')
ylabel('T: thrust')
zlabel('Q: torque')

figure('OuterPosition',[scrsz(3)/2+12 45 scrsz(3)/2-12 scrsz(4)/2],'Name','Grafico RPM','NumberTitle','off')
surf(range_J,range_T,specchio_rpm(2:end,2:end));
xlabel('J: advance ratio')
ylabel('T: thrust')
zlabel('rpm')

disp(['Massima velocità raggiunta: ',num2str(max(max(specchio_rpm(2:end,2:end))))]);

spinta = interp2(specchio_rpm(2:end,1),specchio_rpm(1,2:end), specchio_rpm(2:end,2:end)', range_T, 0.1);
% spinta = [range_T',spinta'];

p1 = 2.038e-7;
p2 = -0.0003129;
p3 = 0.2656;
cont=0;
for s=0:0.1:10
    cont = cont+1;
    sol(cont,1) = s;
    sol(cont,2) = (-p2 + sqrt(abs(p2^2-4*p1*(p3-s))))/2/p1;
    sol(cont,3) = (-p2 - sqrt(abs(p2^2-4*p1*(p3-s))))/2/p1;
end


% save('C:\Users\Dario\Dropbox\Robotica\Quadricottero\Librerie\tabellaRPM',)

clear J Q T col flag_error g grado_max grado_x grado_xQ grado_y grado_yQ k scrsz range_J range_T coeff
clear matrice_coefficientiRPM matrice_coefficientiQ riga rpm rpm_formatted rpm_set stpJ stpT termine_noto