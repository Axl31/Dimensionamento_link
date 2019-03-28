function [p, theta] = calcola_punti_traiettoria(xd, yd, phi)
 %FUNZIONE UTILIZZATA PER IL MAIN PER OTTIMIZZARE I LINK   
distanze_punti = []; %inizializzo il vettore per calcolare la distanza cartesiana dei punti caratterizzanti la traiettoria
%in considerazione
    for i = 1 : length(xd) %faccio un ciclo for con un numero di iterazioni pari ai punti della traiettoria
        dist_punto = sqrt(xd(i)^2 + yd(i)^2); %calcolo la distanza euclidea dall'origine
        distanze_punti = cat(1, distanze_punti, dist_punto); %le salvo nel vettore delle distanze
    end
    
    [~, max_ind] = max(distanze_punti); %salvo l'indice del punto con distanza massima
    [~, min_ind] = min(distanze_punti); %salvo l'indice del punto con distanza minima
    %Traiettoria di esempio nello spazio operativo
    p=[ 30 20;
        xd(max_ind) yd(max_ind);
        20 20;
        xd(min_ind) yd(min_ind)];

    theta=[0;
        phi(max_ind);
        pi/6;
        phi(min_ind)];
end