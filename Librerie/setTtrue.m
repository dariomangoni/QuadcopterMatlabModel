function [T,Q] = setTtrue(rpm_set,J)

load 'C:\Users\Dario\Dropbox\Robotica\Quadricottero\ModelloBasePercorso\APC-SF-10x4.7\matrice_coeffMETRICmod.mat'
load 'C:\Users\Dario\Dropbox\Robotica\Quadricottero\Librerie\max_thrust.mat'

% in base alla matrice dei coefficienti stabilisce il grado del polinomio:
% ogni riga rappresenta un grado della X e ogni colonna della Y;
grado_x = size(matrice_coefficientiRPM,1)-1;
grado_y = size(matrice_coefficientiRPM,2)-1;

grado_xQ = size(matrice_coefficientiQ,1)-1;
grado_yQ = size(matrice_coefficientiQ,2)-1;

% Calcolo Q corrispondente a rpm_set
T = 0;
for riga=0:grado_x
    for col=0:grado_y
        T = T + matrice_coefficientiRPM(riga+1,col+1)*rpm_set^riga*J^col;
    end
end


% Calcolo Q corrispondente a rpm_set
Q = 0;
for riga=0:grado_xQ
    for col=0:grado_yQ
        Q = Q + matrice_coefficientiQ(riga+1,col+1)*rpm_set^riga*J^col;
    end
end