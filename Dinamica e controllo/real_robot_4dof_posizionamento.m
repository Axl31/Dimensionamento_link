close all
clear all
clc

%% Lego EV3 Mindstorm initialization

mylego = legoev3('USB');% vado a dire che il mio dispositivo è collegato
% tramite USB al computer


mymotor1 = motor(mylego, 'A');% specifico L'ID della mia unità di controllo e la porta a cui è collegato
mymotor2 = motor(mylego, 'B');
mymotor3 = motor(mylego, 'C');
mymotor4 = motor(mylego, 'D');

resetRotation(mymotor1);% resetto gli encoder
resetRotation(mymotor2);
resetRotation(mymotor3);
resetRotation(mymotor4);

clearLCD(mylego);% cancello tutto quello che ho sullo schermo


state = true; % variabile di controllo del loop che faccio dopo. E' legata al tasto up dell'unità
% di controllo
controllo= true; % variabile di controllo realizzato da noi.Serve a concludere il ciclo non appena
% raggiunta la configurazione a regime 
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
start_time=tic;% prende il valore temporale nell'istante di tempo in cui 
%viene eseguita la riga di codice

% posizione iniziale. Manipolatore steso
Q0=[0, 0, 0, 0]; 


a1=15;
a2=16;
a3=15;
a4=7;
XY_iniziale=[35 25 0]; % il nostro punto B in cm
XYf=[35 25 pi/6];% è sempre il nostro punto B ma con l'orientamento finale

    
% devo mettere il primo valore di q4 usato per il calcolo della traiettoria
q4=-1.3963;
% valori dei giunti in configurazione iniziale
Q1=cinematica_inversa_4gdl(XY_iniziale(1:2),XY_iniziale(3),[a1,a2,a3,a4],q4)';
Qides=[0; 0; 0;0];% integrale della posizione desiderata
Qi=[0; 0; 0;0]; % integrale della posizione effettiva

q1_prev=Q0(1);% posizione del giunto nell'istante precedente. In questo caso all'inizio sono tutte pari a 0.
q2_prev=Q0(2);
q3_prev=Q0(3);
q4_prev=Q0(4);

% inizializzo le variabili per salvare i dati
Q_=[];
Qdes_=[];

tf=5;% abbiamo messo per il posizionamento 5 secondi


%% Control loop
while(state && controllo) 
    % termino il ciclo quando state diventa false (e lo faccio quando premo
    % un pulsante) o quando controllo diventa false ( e lo faccio quando i
    % giunti sono vicini al loro valore di regime)
  
    time=toc(start_time);
    T(i)=time;
    dt=T(i)-T(i-1);
    
    
    %% Joint trajectory planner
    if time<tf
     % calcolo la posizione e la velocità desiderate per i 4 giunti 
     [qdes1, qdotdes1]=joint_planner(Q0(1), Q1(1), 0, tf, time); % polinomio del quinto ordine
     [qdes2, qdotdes2]=joint_planner(Q0(2), Q1(2), 0, tf, time); 
     [qdes3, qdotdes3]=joint_planner(Q0(3), Q1(3), 0, tf, time); 
     [qdes4, qdotdes4]=joint_planner(Q0(4), Q1(4), 0, tf, time); 
                  
    end
    
    Qdes=[qdes1; qdes2; qdes3; qdes4];
    Qdotdes=[qdotdes1; qdotdes2; qdotdes3; qdotdes4];
    Qides=Qides+Qdes*dt; % integrale della posizione desiderata
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
    K=[30 60 67 5];
    D=[0.01 0 0 0.05];
    I=[0.15 0.15 0.15 0];
    
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
    
    % controllo realizzato da noi: se la posizione desiderata ed effettiva
    % di tutti e quattro i giunti si differenziano di al massimo 2° il
    % manipolatore si ferma. 
    if (abs(q1-Q1(1))<deg2rad(2) && abs(q2-Q1(2))<deg2rad(2) && abs(q3-Q1(3))<deg2rad(2) && ...
            abs(q4-Q1(4))<deg2rad(2))
            controllo=false;
            stop(mymotor1);
            stop(mymotor2);
            stop(mymotor3);
            stop(mymotor4);
    end
    
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

