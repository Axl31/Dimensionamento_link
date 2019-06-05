% Bisogna far partire questo script subito dopo
% real_robot_4dof_posizionamento e non bisogna resettare gli encoder perchè
% gli angoli di giunto che vengono forniti sono calcolati rispetto al
% manipolatore steso.
clearLCD(mylego);% cancello tutto quello che ho sullo schermo
close all; % cosi chiudiamo i grafici aperti su real_robot_4dof_posizionamento
clc;

% eliminiamo queste variabili perchè qui cambiano dimensione. 
clear Q_;
clear Qdes_;
clear T;

state = true; % variabile di controllo del loop che faccio dopo. E' legata al tasto up dell'unità
% di controllo

% imposto le velocità iniziali dei motori  
mymotor1.Speed =0.0;
mymotor2.Speed =0.0;
mymotor3.Speed =0.0;
mymotor4.Speed =0.0;


start(mymotor1);
start(mymotor2);
start(mymotor3);
start(mymotor4);


%% Variables initialization

T(1)=0.0;
time=0.0;
i=2;
start_time=tic;

a1=15;
a2=16;
a3=15;
a4=7;

XY_iniziale=[35 25 0]; % il nostro punto B in cm
XYf=[35 25 pi/6];% è sempre il nostro punto B ma con l'orientamento finale

    
% devo mettere il primo valore di q4 usato per il calcolo della traiettoria
q4=-1.3963;
Q0=cinematica_inversa_4gdl(XY_iniziale(1:2),XY_iniziale(3),[a1,a2,a3,a4],q4)';
Qd=Q0; 
Qides=[0; 0; 0;0];% integrale della posizione desiderata
Qi=[0; 0; 0;0]; % integrale della posizione effettiva

q1_prev=Q0(1);% posizione del giunto nell'istante precedente
q2_prev=Q0(2);
q3_prev=Q0(3);
q4_prev=Q0(4);

Q_=[];
Qdes_=[];

tf=15;% tempo impiegato per eseguire il task
[punto, tempo, PHI, circonferenza1, circonferenza2]=inizializza_simulazione();

%% Control loop
while(state) % termino il ciclo quando state diventa false e lo faccio quando premo il pulsante
    % up dell'unità di controllo
   time=toc(start_time);
    T(i)=time;
    dt=T(i)-T(i-1);
    
    %% Joint trajectory planner
    if time<tf
      [xd,xdd,phi,phit]=planner_TOAD(punto, tempo, PHI, time, circonferenza1, circonferenza2);
      Qddot =  inv_cin_psd(Qd,[xd,phi]',[xdd,phit]',[a1, a2, a3, a4]); %psd sta per pseudo-inversa.
      % Qddot è la velocità desiderata
      Qd=Qd+Qddot*dt; % posizione desiderata
      
      qdes1=Qd(1);
      qdes2=Qd(2);
      qdes3=Qd(3);
      qdes4=Qd(4);
      qdotdes1=Qddot(1);
      qdotdes2=Qddot(2);
      qdotdes3=Qddot(3);
      qdotdes4=Qddot(4);

        
    end
    
    Qdes=[qdes1; qdes2; qdes3; qdes4];
    Qdotdes=[qdotdes1; qdotdes2; qdotdes3; qdotdes4];
    Qides=Qides+Qdes*dt;
    Qdes_=[Qdes_;Qdes'];
    
    %% Stop simulation 
    % perchè se il robot impazzisce posso fermarlo premendo il pulsante in
    % alto dell'unità di controllo
    up=readButton(mylego,'up');
    if up
        state=false;
        stop(mymotor1);
        stop(mymotor2);
        stop(mymotor3);
        stop(mymotor4);
    end
    
    %% Read motor angles
    % leggo gli angoli dall'encoder
    q1= deg2rad(double(readRotation(mymotor1)))/2;
    q2= deg2rad(double(readRotation(mymotor2)))/2;
    q3= deg2rad(double(readRotation(mymotor3)))*(20/36);
    q4= deg2rad(double(readRotation(mymotor4)))*(20/36);
    
    Q=[q1; q2; q3; q4];
    Q_=[Q_;Q'];
    
    
    % derivata della posizione effettiva ( velocità effettiva)
    q1_dot=(q1-q1_prev)/dt;
    q2_dot=(q2-q2_prev)/dt;
    q3_dot=(q3-q3_prev)/dt;
    q4_dot=(q4-q4_prev)/dt;
    
    Qdot=[q1_dot; q2_dot; q3_dot; q4_dot];
    
    Qi=Qi+Q*dt; % integrale della posizione nello spazio dei giunti che ci serve per il controllo PID
    
    
    %% PID controller    
    K=[1 1 1 1]; 
    D=[0 0 0 0];
    I=[0 0 0 0]; 
    command=PID_controller(Q, Qdot, Qi, Qdes, Qdotdes, Qides, K, D, I);
    % command sarà di fatto una velocità mentre in simulazione è una
    % coppia. 
    
    % passo ai motori una velocità  
    mymotor1.Speed = command(1);
    mymotor2.Speed = command(2);
    mymotor3.Speed = command(3);
    mymotor4.Speed = command(4);
    
    %% Write joint angles on the brick LCD
    % scrivo il valore delle variabili di giunto sullo schermo dell'unità
    % di controllo
    writeLCD(mylego,'angle1=',2,1)
    str_f = sprintf('%0.5f',q1);
    writeLCD(mylego,str_f,2,8)
    
    writeLCD(mylego,'angle2=',3,1)
    str_f = sprintf('%0.5f',q2);
    writeLCD(mylego,str_f,3,8)
    
    writeLCD(mylego,'angle3=',4,1)
    str_f = sprintf('%0.5f',q3);
    writeLCD(mylego,str_f,4,8)  
    
    writeLCD(mylego,'angle4=',5,1)
    str_f = sprintf('%0.5f',q4);
    writeLCD(mylego,str_f,5,8)
    
    %% Save previous joint angles    
    % aggiorno il valore precedentemente assunto dai giunti 
    q1_prev=q1;
    q2_prev=q2;
    q3_prev=q3;
    q4_prev=q4;
    
    i=i+1;
    
end

%% Downsampling
% sottocampiono per non saturare la scheda grafica
sample_number=200;

Q_=Q_(1:end/sample_number:end,:);
Qdes_=Qdes_(1:end/sample_number:end,:);
T=T(1:(end-1)/sample_number:end-1);

%% Plot
    figure(1)
    hold on    
    plot(T,Qdes_(:,1),'-r','Linewidth',4)    
    plot(T,Q_(:,1),'-b','Linewidth',4)
    title('Andamento desiderato Vs effettivo giunto 1');
    legend('Q desiderato', 'Q effettivo');
    %  plot(time,q4,'*g','Linewidth',4)
    xlabel('Tempo [s]')
    
    figure(2)
    hold on
    plot(T,Qdes_(:,2),'-r','Linewidth',4)
    plot(T,Q_(:,2),'-b','Linewidth',4)
    title('Andamento desiderato Vs effettivo giunto 2');
    legend('Q desiderato', 'Q effettivo');
    
    figure(3)
    hold on
    plot(T,Qdes_(:,3),'-r','Linewidth',4)
    plot(T,Q_(:,3),'-b','Linewidth',4)   
    title('Andamento desiderato Vs effettivo giunto 3');
    legend('Q desiderato', 'Q effettivo');
    
    
    figure(4)
    hold on
    plot(T,Qdes_(:,4),'-r','Linewidth',4)
    plot(T,Q_(:,4),'-b','Linewidth',4)
    title('Andamento desiderato Vs effettivo giunto 4');
    legend('Q desiderato', 'Q effettivo');

