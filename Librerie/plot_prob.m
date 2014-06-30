% % Disegno della distribuzione normale, calcolo media e varianza
% clear all
% clc
% close all
% 
% % load 'C:\Users\Dario\Dropbox\Robotica\Quadricottero\TestS\TestSensoristica\ProvaComplessivaAccelerometer.mat'
% % distribuzione=Dati_A(1:250,1)';
% 
% load C:\Users\Dario\Dropbox\Robotica\Quadricottero\TestS\campomagnetico.mat
% distribuzione = cacca;

function [media, valore_probabile, varianza] = plot_prob(dati)

for s=1:size(dati,2)
    distribuzione = dati(1:end,s);
    num_campioni = length(distribuzione);
    numero_suddivisioni = 1000; % risoluzione del grafico
    dens_prob = zeros(2,numero_suddivisioni); % densità di probabilità dei diversi valori
    step_distribuzione = ( max(distribuzione) - min(distribuzione) ) /numero_suddivisioni; % intervallo di valori per cui un dato si può ritenere parte di quello step
    media=0;
    for k=1:num_campioni
        if distribuzione(k)==max(distribuzione) % se si sta osservando il valore massimo della distribuzione lo si mette nell'ultimo indice
            indice=numero_suddivisioni;
        else
        indice = fix( ( distribuzione(k) - min(distribuzione) )/step_distribuzione)+1; % lo si mette nell'intervallo in cui rientra per difetto
        end
        dens_prob(2,indice)= dens_prob(2,indice) + 1/num_campioni;

        media = media + distribuzione(k);

    end
    % sum(dens_prob(2,:)) % dovrebbe essere 1 se non ci sono errori nel programma
    media = media/num_campioni;

    [prob_max,indice] = max(dens_prob(2,:));
    dens_prob(1,:) = min(distribuzione)+0.5*step_distribuzione : step_distribuzione : max(distribuzione)-0.5*step_distribuzione;
    valore_probabile = dens_prob(1,indice);
    figure
    plot( dens_prob(1,:) ,dens_prob(2,:) );
    xlim([ min(distribuzione) max(distribuzione) ])
    y_limite = get(gca, 'YLim');
    line([media media],[0 1],'Color','red','LineWidth',2)
    ylim(y_limite)

    disp(['La media è ',num2str(media)]);
    disp(['Il valore più probabile è ',num2str(valore_probabile),' con una probabilità del ',num2str(prob_max),'(max 1)\n']);

    varianza = 0;
    for k=1:num_campioni
        varianza = varianza + ( distribuzione(k) - media )^2;
    end
    varianza = varianza/num_campioni;
    disp(['La varianza è ',num2str(varianza),'; quindi la deviazione standard ',num2str(sqrt(varianza))]);
end

end
