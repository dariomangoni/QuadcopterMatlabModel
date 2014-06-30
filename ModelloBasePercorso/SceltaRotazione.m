close all
clear all
clc

range_partenza = 0:pi/10:2*pi;
range_target = range_partenza;

rotazione_scelta = zeros(numel(range_partenza),numel(range_partenza));
stpP = 1;
for angolo_partenza= range_partenza
    stpT = 1;
    for angolo_target = range_target
        partenza = atan2(sin(angolo_partenza),cos(angolo_partenza));
        target = atan2(sin(angolo_target),cos(angolo_target));

        scelta(1) = target - ( 2*pi + partenza);
        scelta(2) = target - partenza;
        scelta(3) = (2*pi + target) - ( 2*pi + partenza);
        scelta(4) = (2*pi + target) - partenza;

        [~,indice] = min (abs(scelta));

        rotazione_scelta(stpP,stpT) = scelta(indice)*180/pi;
        stpT = stpT +1;
    end
    stpP = stpP+1;
end


%% Disegno grafico
surf(range_partenza.*180./pi,range_partenza.*180./pi,rotazione_scelta);
