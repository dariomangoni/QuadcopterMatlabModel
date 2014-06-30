function [rpm_set, Q, T] = setRPM(J,T)

if T<=0
    T = 0;
    rpm_set = 0;
    Q=0;
    return
end

flag_Q = false;

load 'C:\Users\Dario\Dropbox\Robotica\Quadricottero\ModelloBasePercorso\APC-SF-10x4.7\matrice_coeffMETRICmod.mat'
load 'C:\Users\Dario\Dropbox\Robotica\Quadricottero\Librerie\max_thrust.mat'

% in base alla matrice dei coefficienti stabilisce il grado del polinomio:
% ogni riga rappresenta un grado della X e ogni colonna della Y;
grado_x = size(matrice_coefficientiRPM,1)-1;
grado_y = size(matrice_coefficientiRPM,2)-1;
coeff=zeros(grado_x+1,1);

grado_xQ = size(matrice_coefficientiQ,1)-1;
grado_yQ = size(matrice_coefficientiQ,2)-1;

% non c'è una formula diretta per calcolare gli rpm dato J e T perchè la
% soluzione non è unica!
% l'interpolazione vuole T = f(J,rpm)
% l'interpolazione buona si ottiene con un polinomio di quarto grado ancora
% risolvibile analiticamente bisogna però invertire l'equazione per avere
% 
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
    if abs(rpm(k,2)) > 0.05
        rpm_formatted(k,1) = NaN;
    end
    if (rpm(k,1)<0) || (rpm(k,1)>10000)
        rpm_formatted(k,1) = NaN;
    end
end

flag_error = 0;
for k=1:grado_x
    if ~isnan(rpm_formatted(k,1))
        rpm_set = rpm_formatted(k,1);
        flag_error=flag_error+1;
    end
end

if flag_error~=1
    if T<8
        [rpm_set, Q, T] = setRPM(J+0.05,T);
        flag_Q = true;
    else
    T_max = interp1(max_thrust(:,1),max_thrust(:,2),J,'linear',0);
    rpm_set = interp1(max_thrust(:,1),max_thrust(:,3),J,'linear',0);
    disp(['Errore per J=',num2str(J),' e T=',num2str(T),' con ',num2str(flag_error),' velocità trovate']);
    disp(['Per un valore di J=',num2str(J),' la massima T=',num2str(T_max),' è raggiunta a ',num2str(rpm_set),' rpm']);
    T=T_max;
    end
end


% Calcolo Q corrispondente a rpm_set
if flag_Q==false
Q = 0;
for riga=0:grado_xQ
    for col=0:grado_yQ
        Q = Q + matrice_coefficientiQ(riga+1,col+1)*rpm_set^riga*J^col;
    end
end
end
