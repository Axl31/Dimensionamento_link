function [p, theta] = calcola_punti_traiettoria(xd, yd, phi)
    % [p, theta] = calcola_punti_traiettoria(xd, yd, phi)
    % FUNZIONE UTILIZZATA PER IL MAIN PER OTTIMIZZARE I LINK
    % restituisce le coordinate (posizione e orientamento) dei punti A, B e
    % i punti a massima e minima distanza dall'origine
    distanze_punti = zeros(length(xd),1); %inizializzo il vettore per calcolare la distanza cartesiana dei punti
    %caratterizzanti la traiettoria
    
    for i = 1 : length(xd) %faccio un ciclo for con un numero di iterazioni pari ai punti della traiettoria
        dist_punto = sqrt(xd(i)^2 + yd(i)^2); %calcolo la distanza euclidea dall'origine
        distanze_punti(i) = dist_punto; %le salvo nel vettore delle distanze
    end
    
    [~, max_ind] = max(distanze_punti); %salvo l'indice del punto con distanza massima
    [~, min_ind] = min(distanze_punti); %salvo l'indice del punto con distanza minima
    
    p=[ xd(1) yd(1); %punto B
        xd(max_ind) yd(max_ind); % punto massima distanza
        xd(floor(length(xd)/3))  yd(floor(length(yd)/3)); % punto A (length(yd)/3) perchè A è 1/3 della
        % traiettoria)
        xd(min_ind) yd(min_ind)]; % punto a minima distanza

    theta=[phi(1);
        phi(max_ind);
        phi(floor(length(phi)/3));
        phi(min_ind)];
end