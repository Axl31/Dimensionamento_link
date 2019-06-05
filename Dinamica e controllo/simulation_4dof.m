function [Q_] = simulation_4dof()
    %[Q_] = simulation_4dof()
    % simulazione dell'esecuzione della traiettoria
    
    %% Variables initialization
    a1=15;
    a2=16;
    a3=15;
    a4=7;
    

    XY_iniziale=[35 25 0]; % il nostro punto B in cm
    XYf=[35 25 pi/6];% è sempre il nostro punto B ma con l'orientamento finale

    Qides=[0;0;0;0];
    % devo mettere il primo valore di q4 usato per il calcolo della traiettoria
    q4=-1.3963;
    Qdes=cinematica_inversa_4gdl(XY_iniziale(1:2),XY_iniziale(3),[a1,a2,a3,a4],q4)'; 
    % serve per calcolare i primi valori dei giunti
    Qdotdes=[0;0;0;0]; % velocità
    Q2dotdes=[0;0;0;0]; % accelerazioni

    Qi=[0;0;0;0]; % integrale della posizione
    Q=Qdes; % inizializzazione. In questa maniera diciamo che la posizione desiderata coincide
    %con quella effettiva
    Qdot=[0;0;0;0]; % velocità effettiva
    Q2dot=[0;0;0;0]; % accelerazione effettiva

    tau=[0;0;0;0]; %coppia
    % inizializziamo le variabili dove salavre tutti i dati
    Q2dot_=[]; 
    Qdot_=[];
    Q_=[];

    XY1=[];
    XY2=[];
    XY3=[];
    XY4=[];

    Qdotdes_=[];
    Qdes_=[];


    XY_err_IK=[];

    tf=15;% è il tempo totale previsto per l'esecuzione della traiettoria
    [punto, tempo, phi, circonferenza1, circonferenza2] = inizializza_simulazione();

    T(1)=0.0;
    time=0.0;
    i=2;
    start_time=tic;% prende il valore temporale nell'istante di tempo in cui viene eseguita la riga di codice
    

    %% Control loop

    while(time<tf)
        time=toc(start_time);% tempo corrente.(In poche parole prendo il tempo che trascorre nell'esecuzione 
        % tra il comando tic e toc)
        T(i)=time;
        dt=T(i)-T(i-1);

        %% Online Trajectory planning   
        [xd,xdd,phi,phit]=planner_TOAD(punto, tempo, phi, time, circonferenza1, circonferenza2);
        XY_=xd; % posizione
        XYdot_=xdd; % derivata della posizione
        XY=[XY_'; phi];% posa 
        XYdot=[0;0;0]; % derivata della posa
        %% Online inverse kinematics

        Q_dotdes=inv_cin_psd(Qdes,XY,XYdot,[a1,a2,a3,a4]);%psd sta per pseudo-inversa
        Qdes=Qdes+Q_dotdes*dt;
        %Q_dotdes=inv_man_rid(Q,XY,XYdot,a1,a2,a3);
        %Qdes=Q+Q_dotdes*dt;
        Qides=Qides+Qdes*dt;

        XY_IK=cinematica_diretta_4gdl(Qdes,[a1,a2,a3,a4]);
        XY_err_IK=[XY_err_IK; XY_IK(1:3)'-XY(1:3)'];%XY_IK è quella desiderata,l'altra quella effettiva
        %% PID controller
        
%         K=[1 1 1 1];
%         D=[0.35 0.1 0.1 0.08];
%         I=[0.4 0.35 0.1 0];


        K=[9000 1000 1000 300];% 18500[16900 2000 1000 300]%[50000 15000 1000 20]
        D=[0 0 0 0];%[0 0 0 100]
        I=[0 0 0 0];


        tau= PID_controller(Q, Qdot, Qi, Qdes, Qdotdes, Qides, K, D, I);
        %% Robot dynamic model
        
        Q2dot=dynamic_model_4dof(tau,Q, Qdot);% accelerazione effettive dei giunti
        Qdot=Qdot+Q2dot*dt; % velocità effettive dei giunti
        Q=Q+Qdot*dt; % posizione effettiva dei giunti
        Qi=Qi+Q*dt;%integrale della posizione attuale


        %% Real variable saving
        Q2dot_=[Q2dot_;Q2dot'];
        Qdot_=[Qdot_;Qdot'];
        Q_=[Q_;Q'];

        [xy1, xy2, xy3 xy4]=kin_man_rid_progetto(Q,[a1,a2,a3,a4]);

        XY1=[XY1; xy1'];
        XY2=[XY2; xy2'];
        XY3=[XY3; xy3'];
        XY4=[XY4; xy4'];

        %% Desired variable saving

        Qdotdes_=[Qdotdes_;Qdotdes'];
        Qdes_=[Qdes_;Qdes'];

        i=i+1;
    end

    %% Downsampling
    % sottocampiono per poterli plottare 
    sample_number=200;

    Q_=Q_(1:end/sample_number:end,:);
    Qdes_=Qdes_(1:end/sample_number:end,:);
    XY_err_IK=XY_err_IK(1:end/sample_number:end,:);
    T=T(1:(end-1)/sample_number:end-1);

    %% Plot
    figure(1)
    %subplot(4,1,1)
    plot(T,Q_(:,1),'-b','Linewidth',4)
    hold on
    plot(T,Qdes_(:,1),'-r','Linewidth',4)
    title('Andamento effettivo Vs desiderato giunto 1');
    legend('Q effettivo', 'Q desiderato');

    figure(2)
    %subplot(4,1,2)
    plot(T,Q_(:,2),'-b','Linewidth',4)
    hold on
    plot(T,Qdes_(:,2),'-r','Linewidth',4)
    title('Andamento effettivo Vs desiderato giunto 2');
    legend('Q effettivo', 'Q desiderato');

    figure(3)
    %subplot(4,1,3)    
    plot(T,Q_(:,3),'-b','Linewidth',4)
    hold on
    plot(T,Qdes_(:,3),'-r','Linewidth',4)
    title('Andamento effettivo Vs desiderato giunto 3');
    legend('Q effettivo', 'Q desiderato');

    figure(4)
    %subplot(4,1,4)
    plot(T,Q_(:,4),'-b','Linewidth',4)
    hold on
    plot(T,Qdes_(:,4),'-r','Linewidth',4)
    title('Andamento effettivo Vs desiderato giunto 4');
    legend('Q effettivo', 'Q desiderato');


    figure(5)
    subplot(3,1,1)
    plot(T,XY_err_IK(:,1),'-b','Linewidth',4)
    title(' Errore su x');

    subplot(3,1,2)
    plot(T,XY_err_IK(:,2),'-b','Linewidth',4)
    title(' Errore su y');

    subplot(3,1,3)
    plot(T,XY_err_IK(:,3),'-b','Linewidth',4)
    title(' Errore orientamento');

    %%
    MAKE_VIDEO = 1;
    if(MAKE_VIDEO)
        motion = VideoWriter(['mov_2D_',datestr(now,30),'.avi']);
        open(motion);
    end


    figure(6)
    title('Simulazione movimento robot')
    passo = 40;
    for i=1:passo:size(XY1,1)
        axis equal
        plot(XY(1:passo:end,1), XY(1:passo:end,2),'-k','Linewidth',4)

        hold on
        plot([0 XY1(i,1)],[0 XY1(i,2)],'-r','Linewidth',4)    
        plot([XY1(i,1) XY2(i,1)],[XY1(i,2) XY2(i,2)],'-b','Linewidth',4)
        plot([XY2(i,1) XY3(i,1)],[XY2(i,2) XY3(i,2)],'-g','Linewidth',4)
        plot([XY3(i,1) XY4(i,1)],[XY3(i,2) XY4(i,2)],'-m','Linewidth',4)
        axis equal

        if(MAKE_VIDEO)
            F = getframe(gcf);
            writeVideo(motion,F);
        end
        hold off

    end



    if(MAKE_VIDEO)
        close(motion);
    end
end
