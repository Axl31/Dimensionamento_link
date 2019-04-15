clear all
close all
clc

%% Variables initialization
a1=0.15;
a2=0.16;
a3=0.15;
a4=0.07;

XY_iniziale=[0.02 0.21 1.047]; % il nostro punto B in metri
XYf=[0.2198 0.1315 0.953];% è sempre il nostro punto B ma con l'orientamento finale

Qides=[0;0;0;0];
% devo mettere il primo valore di q4 usato per il calcolo della traiettoria
q4=deg2rad(-91);
Qdes=cinematica_inversa_4gdl(XY_iniziale(1:2),XY_iniziale(3),[a1,a2,a3,a4],q4)';
Qdotdes=[0;0;0;0];
Q2dotdes=[0;0;0;0];

Qi=[0;0;0;0];
Q=Qdes;
Qdot=[0;0;0;0];
Q2dot=[0;0;0;0];

tau=[0;0;0;0];
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

tf=5;%% mettere il fine della simulazione della traiettoria( dovrebbero essere 15s)

T(1)=0.0;
time=0.0;
i=2;
start_time=tic;

%% Control loop

while(time<tf)
    time=toc(start_time);% qui uso il tempo reale e non il tempo di ciclo come nel caso della
    %pianificazione di traiettoria
    T(i)=time;
    dt=T(i)-T(i-1);
    
    %% Online Trajectory planning
    IN_XY(1:2)=XY_iniziale(1:2);
    IN_XY(3:4)=XYf(1:2);
    IN_XY(5)=0;
    IN_XY(6)=tf;
    IN_XY(7)=time;    
    [punto, tempo, phi, circonferenza1, circonferenza2] = inizializza_simulazione();
    [xd,xdd,phi,phit]=planner_TOAD(punto, tempo, phi, time, circonferenza1, circonferenza2);
    XY_=xd;
    XYdot_=xdd;
    %theta=pi/3;
    XY=[XY_';XY_iniziale(3)];
    XYdot=[0;0;0];
    %% Online inverse kinematics
    
    Q_dotdes=inv_cin_psd(Qdes,XY,XYdot,[a1,a2,a3,a4]);
    Qdes=Qdes+Q_dotdes*dt;
    %Q_dotdes=inv_man_rid(Q,XY,XYdot,a1,a2,a3);
    %Qdes=Q+Q_dotdes*dt;
    Qides=Qides+Qdes*dt;
    
    XY_IK=cinematica_diretta_4gdl(Qdes,[a1,a2,a3,a4]);
    XY_err_IK=[XY_err_IK; XY_IK(1:3)'-XY(1:3)'];%XY_IK è quella desiderata,l'altra quella effettiva
    %% PID controller

    K=[1 1 1 1];
    D=[0.35 0.1 0.1 0.08];
    I=[0.4 0.35 0.1 0];
    
    tau= PID_controller(Q, Qdot, Qi, Qdes, Qdotdes, Qides, K, D, I);
    
    %% Robot dynamic model
    Q2dot=dynamic_model_4dof(tau,Q, Qdot);
    Qdot=Qdot+Q2dot*dt;
    Q=Q+Qdot*dt;
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
sample_number=200;

Q_=Q_(1:end/sample_number:end,:);
Qdes_=Qdes_(1:end/sample_number:end,:);
XY_err_IK=XY_err_IK(1:end/sample_number:end,:);
T=T(1:(end-1)/sample_number:end-1);

%% Plot
figure(1)
subplot(4,1,1)
plot(T,Q_(:,1),'-b','Linewidth',4)
hold on
plot(T,Qdes_(:,1),'-r','Linewidth',4)

subplot(4,1,2)
plot(T,Q_(:,2),'-b','Linewidth',4)
hold on
plot(T,Qdes_(:,2),'-r','Linewidth',4)

subplot(4,1,3)
plot(T,Q_(:,3),'-b','Linewidth',4)
hold on
plot(T,Qdes_(:,3),'-r','Linewidth',4)

subplot(4,1,4)
plot(T,Q_(:,4),'-b','Linewidth',4)
hold on
plot(T,Qdes_(:,4),'-r','Linewidth',4)


figure(2)
subplot(3,1,1)
plot(T,XY_err_IK(:,1),'-b','Linewidth',4)

subplot(3,1,2)
plot(T,XY_err_IK(:,2),'-b','Linewidth',4)

subplot(3,1,3)
plot(T,XY_err_IK(:,3),'-b','Linewidth',4)


MAKE_VIDEO = 1;
if(MAKE_VIDEO)
    motion = VideoWriter(['mov_2D_',datestr(now,30),'.avi']);
    open(motion);
end


% figure(3)
% for i=1:40:size(XY1,1)
%     axis equal
%     plot([XY_iniziale(1) XYf(1)],[XY_iniziale(2) XYf(2)],'-k','Linewidth',4) % da sostituire con la nostra...
%     % traiettoria 
%     hold on
%     plot([0 XY1(i,1)],[0 XY1(i,2)],'-r','Linewidth',4)    
%     plot([XY1(i,1) XY2(i,1)],[XY1(i,2) XY2(i,2)],'-b','Linewidth',4)
%     plot([XY2(i,1) XY3(i,1)],[XY2(i,2) XY3(i,2)],'-g','Linewidth',4)
%     axis equal
%     
%     if(MAKE_VIDEO)
%         F = getframe(gcf);
%         writeVideo(motion,F);
%     end
%     %pause(0.5)
%     hold off
%     
% end
% 
% 
% 
% if(MAKE_VIDEO)
%     close(motion);
% end
% 
