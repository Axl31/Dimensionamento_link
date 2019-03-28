function [punto, tempo, phi, circonferenza1, circonferenza2] = inizializza_simulazione()
%FUNZIONE UTILIZZATA PER IL MAIN PER OTTIMIZZARE I LINK
%definisco la struct per i punti A e B
punto = struct;
punto.iniziale = [35 25]; %B
punto.finale = [25 25]; %A
%     alfa_2=deg2rad(180);
%     beta=deg2rad(268.84);
%     r1=5;
%     r2=7;
%     d1=r1*alfa_2;
%     d2=r2*beta;
%vado a definire gli intervalli temporali:
tf=15;
tempo = struct;
tempo.iniziale = 0;
%tempo.finale1 = tf/(1+d2/d1); questa è una variante per avere una
%divisione temporale più omogenea
tempo.finale1 = tf/3;
tempo.finale2 = tf;

%definisco l'orientamento: nota in questo caso il phi iniziale indica
%l'orientamento finale, c'è una discrepanza di notazione.
phi.iniziale = pi/6;
phi.finale = 0;

%definisco i parametri geoemtrici per i due archi di circonferenza:
circonferenza1 = struct;
circonferenza1.centro = [30 25]; %centro circonferenza
circonferenza1.raggio = 5;

circonferenza2 = struct;
circonferenza2.centro = [30 20.1];
circonferenza2.raggio = 7;
end