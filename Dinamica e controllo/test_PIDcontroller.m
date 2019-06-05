% clear all
% close all
% clc

function []= test_PIDcontroller()
    % test_PIDcontroller()
    % simulazione del posizionamento
    %% Variables initialization
    a1=15;
    a2=16;
    a3=15;
    a4=7;
    
    load('joints.mat'); % carico i valori degli angoli di giunto calcolati da check_trajectory_progetto


    Qi=[0;0;0;0];% Q integrata. Integrale della posizione attuale. Lo stiamo solo inizializzando
    Q=[0;0;0;0]; % posizione iniziale manipolatore steso
    Qdot=[0;0;0;0]; % velocità
    Q2dot=[0;0;0;0]; % accelerazione

    Qides=[0;0;0;0]; % integrale della posizione desiderata
    Qdes=[joints(1,1);joints(1,2);joints(1,3);joints(1,4)];% valori angoli di giunto  per il punto B 
    Qdotdes=[0;0;0;0]; % velocità desiderata
    Q2dotdes=[0;0;0;0]; % accelerazione desiderata

    tau=[0;0;0;0]; % coppia
    % inizializziamo le seguenti variabili per poi salvarci tutti i dati
    Q2dot_=[];
    Qdot_=[];
    Q_=[];

     XY_IK1=[];
     XY_IK2=[];
     XY_IK3=[];
     XY_IK4=[];

    T(1)=0.0;
    time=0.0;
    i=2;
    start_time=tic;% prende il valore temporale nell'istante di tempo in cui viene eseguita la riga di codice


    %% Control loop
    while(time<5)  
    time=toc(start_time); % tempo corrente.(In poche parole prendo il tempo che trascorre nell'esecuzione 
% tra il comando tic e toc)
    T(i)=time; 
    dt=T(i)-T(i-1);
    Qides=Qides+Qdes*dt; % integrazione della posizione desiderata

    %% PID controller

    K=[10000 5000 90000 8];%20 10 2 1
    D=[0.1 0 0.1 0];%2 1 0.8 0.4
    I=[0.1 0 0 0];%8 4 0.8 0.4
    
    tau= PID_controller(Q, Qdot, Qi, Qdes, Qdotdes, Qides, K, D, I);
    

    %% Robot dynamic model
    Q2dot=dynamic_model_4dof(tau,Q, Qdot); % utilizzo il modello dinamico diretto per calcolare 
    % l'accelerazione dei giunti
    Qdot=Qdot+Q2dot*dt; % ottengo la velocità integrando l'accelerazione
    Q=Q+Qdot*dt; % ottengo la posizione integrando la velocità
    Qi=Qi+Q*dt; % integro la posizione per l'azione integrativa del controllo

    % Salvo le variabili in un unico vettore per plottarli in seguito
    Q2dot_=[Q2dot_;Q2dot'];
    Qdot_=[Qdot_;Qdot'];
    Q_=[Q_;Q'];

    [xy_ik1, xy_ik2, xy_ik3, xy_ik4]=kin_man_rid_progetto(Q,[a1,a2,a3,a4]);
    % Serve per calcolare la posizione con la cinematica diretta

    % Ci salviamo la posizione di ogni link
        XY_IK1=[XY_IK1; xy_ik1'];
        XY_IK2=[XY_IK2; xy_ik2'];
        XY_IK3=[XY_IK3; xy_ik3'];
        XY_IK4=[XY_IK4; xy_ik4'];

    i=i+1;
    end

    %% Downsampling
    % Riduciamo il numero di campioni per poterli plottare 
    sample_number=200;

    Q_=Q_(1:floor(end/sample_number):end,:); % il floor serve per arrotondare per difetto 
    % cosi da ottenere un indice intero 
    T=T(1:floor((end-1)/sample_number):end-1);

    Qdes1_=ones(length(Q_(:,1)),1)*Qdes(1);
    Qdes2_=ones(length(Q_(:,2)),1)*Qdes(2);
    Qdes3_=ones(length(Q_(:,3)),1)*Qdes(3);
    Qdes4_=ones(length(Q_(:,4)),1)*Qdes(4);

    %% Plot
    figure(1)
    subplot(4,1,1)
    plot(T,Q_(:,1),'-b','Linewidth',4)
    hold on
    plot(T,Qdes1_,'-r','Linewidth',4)
    title('Andamento effettivo Vs desiderato giunto 1');
    legend('Q effettivo', 'Q desiderato');

    subplot(4,1,2)
    plot(T,Q_(:,2),'-b','Linewidth',4)
    hold on
    plot(T,Qdes2_,'-r','Linewidth',4)
    title('Andamento effettivo Vs desiderato giunto 2');
    legend('Q effettivo', 'Q desiderato');

    subplot(4,1,3)
    plot(T,Q_(:,3),'-b','Linewidth',4)
    hold on
    plot(T,Qdes3_,'-r','Linewidth',4)
    title('Andamento effettivo Vs desiderato giunto 3');
    legend('Q effettivo', 'Q desiderato');

    subplot(4,1,4)
    plot(T,Q_(:,4),'-b','Linewidth',4)
    hold on
    plot(T,Qdes4_,'-r','Linewidth',4)
    title('Andamento effettivo Vs desiderato giunto 4');
    legend('Q effettivo', 'Q desiderato');

end
