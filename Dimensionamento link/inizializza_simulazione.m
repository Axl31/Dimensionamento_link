function [punto, tempo, phi, circonferenza1, circonferenza2] = inizializza_simulazione()
%FUNZIONE UTILIZZATA PER IL MAIN PER OTTIMIZZARE I LINK
%definisco la struct per i punti A e B
punto = struct;
punto.iniziale = [35 25]; %B
punto.finale = [25 25]; %A

%vado a definire gli intervalli temporali:
tf=15;
tempo = struct;
tempo.iniziale = 0;
tempo.finale1 = tf/3;
tempo.finale2 = tf;

%definisco l'orientamento.
phi= struct;
phi.iniziale = 0; 
phi.finale = pi/6; 

%definisco i parametri geoemtrici per i due archi di circonferenza:
circonferenza1 = struct;
circonferenza1.centro = [30 25]; %centro circonferenza
circonferenza1.raggio = 5;

circonferenza2 = struct;
circonferenza2.centro = [30 20.1];
circonferenza2.raggio = 7;
end