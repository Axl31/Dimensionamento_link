%funzione main per il dimensionamento dei link seguendo una inversione
%della cinematica geometrica, su un sottoinsieme di punti opportunatamente
%scelti della traiettoria da eseguire. 

clc; clear; close all;
%mi definisco i parametri geometrici della mia traiettoria richiamando la
%funzione 'inizializza_simulazione':
[punto, tempo, phi, circonferenza1, circonferenza2] = inizializza_simulazione();
%'punto' è una struct che contiene i due punti A e B;
%'tempo' è una struct contenente il tempo iniziale, tempo intermedio (nel
%nostro caso il tempo che il manipolatore impiega per giungere dal punto B
%al punto A seguendo il primo arco di circonferenza in senso antiorario, e
%il tempo finale di esecuzione della traiettoria;
%'phi' è una struct che ci da l'orientamento iniziale e finale;
% 'circonferenza'(1/2) ci dà una struct contenente il raggio dell'arco di
% circonferenza e il suo centro. In particolare la circonferenza 1 è la più
% piccola.

%inizializzo i vettori 'xd', ovvero l'ascissa desiderata della traiettoria,
%'yd' l'ordinata desiderata e 'phid' l'orientamento desiderato. I tre
%elementi compongono la posa desiderata dell'organo terminale.
xd = [];
yd = [];
phid = [];
t = []; %inizializzo il vettore dei tempi

dt = 0.05; %dò un passo temporale 0.05

%Creo la possibilità di decidere se pianificare la traiettoria via codice
%matlab (For) oppure utilizzando uno schema Simulink.
prompt = 'Come vuoi pianificare la traiettoria? [F]or - [S]imulink: ';
str = input(prompt,'s');

if strip(lower(str)) == "s" %codice simulink
    puntoS = timeseries([punto.iniziale; punto.finale]);
    tempoS = timeseries([tempo.iniziale; tempo.finale1; tempo.finale2]);
    phiS = timeseries([phi.iniziale, phi.finale]);
    centri_circonferenzeS = timeseries([circonferenza1.centro; circonferenza2.centro]);
    raggi_circonferenzeS = timeseries([circonferenza1.raggio; circonferenza2.raggio]);
    
    simulazione = sim("TOAD_simulation", 'StartTime',num2str(tempo.iniziale),...
        'StopTime',num2str(tempo.finale2),...
        'FixedStep',num2str(dt));
    
    xd = simulazione.XD(:,1);
    yd = simulazione.XD(:,2);
    phid = simulazione.PHI;
    t = simulazione.tout;
else %codice matlab:
    t = (0 : dt : tempo.finale2)'; %definisco il vettore dei tempi
    XD = zeros(length(t), 2); %inizializzo il vettore della posizione desiderata
    XDD = zeros(length(t), 2); %inizializzo il vettore della derivata della posizione desiderata
    PHI = zeros(length(t), 1); %inizializzo il vettore dell'orientamento desiderato
    PHID = zeros(length(t),1); %inizializzo il vettore della derivata dell'orientamento desiderato
    for i = 1 : length(t) %faccio un ciclo con iterazioni date dal passo temporale precedentemente scelto
        %richiamo la funzione 'planner_TOAD' che dati in ingresso le struct
        %'punto', 'tempo', 'phi', 'circonferenza1 e 2' e il tempo relativo
        %all'iterazione considerata, mi dà in uscita la posizione
        %desiderata e l'orientamento, comprese le loro derivate
 [XD(i,:),XDD(i,:), PHI(i),PHID(i)] = ...
            planner_TOAD(punto, tempo, phi, t(i), circonferenza1, circonferenza2); 
        
    end
    %divido la posizione in ordinate ed ascisse
    xd = XD(:,1);
    yd = XD(:,2);
    phid = PHI;
end

%utilizzo la funzione calcola_punti_traiettoria, per avere in uscita delle
%posizione e orientamente opportunatamente scelti per il dimensionamento
%del link, ovvero punti particolari in cui vi è la necessità di verifica
%che il robot possa passarci:
[p, theta] = calcola_punti_traiettoria(xd, yd, phid);

%richiamo quindi la funzione 'ottimizza_link' che date in ingresso le pose
%scelte, mi dà in uscita il link ottimale secondo i funzionali definiti a
%lezione e il vettore delle quaterne di link che permettono al robot di
%passare per i punti precedentemente selezionati
[link,link_scelti] = ottimizza_link(p, theta);
