close all
clear all
clc


%carico i limiti di giunto, servono per la verifica dei link trovati
load('joint_lim2.mat');
%definisco le lunghezze dei link se voglio fare la prova singola
a1=15;
a2=16;
a3=15;
a4=7;
%carico la traiettoria desiderata contenente anche il vettore link_scelti
load('traiettoria_giusta2.mat');
%definisco un ciclo for per provare ogni link che ho trovato da
%ottimizza_link
% for z=1:size(link_scelti)
% %definisco le dimensioni di link prese in considerazione
% a1=link_scelti(z,1);
% a2=link_scelti(z,2);
% a3=link_scelti(z,3);
% a4=link_scelti(z,4);


tf=15; %definisco il tempo finale
dt=tf/size(XY,1); %definisco il passo temporale
T=(0:dt:tf-dt); %definisco il vettore dei tempi

XYdot=[0,0,0; diff(XY,1,1)/dt];
% fissiamo un q4 e sfruttiamo la funzione cinematica_inversa_4DoF che
% abbiamo usato per la traiettoria. Tanto questo passaggio ci serve solo
% per calcolare le posizioni iniziali con cui far partire il manipolatore,
% successivamnete implementiamo l'algoritmo di inversione cinematica.
trovato = false;
resolution_q=deg2rad(15);
q4 = joint_lim(4,1)+(deg2rad(5));                    
while q4 <= joint_lim(4,2)-(deg2rad(5)) && ~trovato
    Q0 = cinematica_inversa_4gdl(XY(1,1:2),XY(1,3),[a1,a2,a3,a4], q4)';
    if (~isempty(Q0) && joint_lim(1,1)+(deg2rad(5)) <= Q0(1) && Q0(1) <= joint_lim(1,2)-(deg2rad(5))...
                    && joint_lim(2,1)+(deg2rad(5)) <= Q0(2) && Q0(2) <= joint_lim(2,2)-(deg2rad(5))...
                    && joint_lim(3,1)+(deg2rad(5)) <= Q0(3) && Q0(3) <= joint_lim(3,2)-(deg2rad(5))...
                    && joint_lim(4,1)+(deg2rad(5)) <= Q0(4) && Q0(4) <= joint_lim(4,2)-(deg2rad(5)))
        trovato = true;
        Q=Q0;
    end
    q4 = q4 + resolution_q;
end

%inizializzo i vettori contenenti la posa di ciascun giunto
XY_IK1=[];
XY_IK2=[];
XY_IK3=[];
XY_IK4=[];
XY_err=[]; %il vettore errore
joints=[]; %il vettore dove andrò ad inserire le variabili di giunto per ogni punto della traiettoria 


for i=1:size(XY,1)
    
    % Inversione cinematica
    Q_dot=inv_cin_psd(Q,XY(i,1:3)',XYdot(i,1:3)',[a1, a2, a3, a4],joint_lim);
    Q=Q+Q_dot*dt; %integro la Q_dot
    
    joints=[joints;Q']; %mi salvo il vettore contenente i parametri di giunto
    %utilizzo la cinematica diretta per ricondurmi alla posa di ogni
    %giunto:
    [xy_ik1, xy_ik2, xy_ik3, xy_ik4]=kin_man_rid_progetto(Q,[a1, a2, a3, a4]);
    
    XY_IK1=[XY_IK1; xy_ik1'];
    XY_IK2=[XY_IK2; xy_ik2'];
    XY_IK3=[XY_IK3; xy_ik3'];
    XY_IK4=[XY_IK4; xy_ik4'];
    
    
    XY_err=[XY_err; xy_ik4(1:3)'-XY(i,1:3)];
end
%vado a definirmi il vettore minimoq4, in cui mi salvo per ogni
%combinazione il valore minimo che q4 tocca
% minimoq4(z)=min(joints(:,4));
% end

    




figure(1)
%subplot(4,1,1)
%plot(T,rad2deg(joints(:,1)),'-b','Linewidth',4)
plot(T,joints(:,1),'-b','Linewidth',4)
title('Andamento giunto 1')
xlabel('Tempo [s]')
ylabel('Angolo [rad]')
grid on
grid minor

figure(3)
%subplot(4,1,2)
%plot(T,rad2deg(joints(:,2)),'-b','Linewidth',4)
plot(T,joints(:,2),'-b','Linewidth',4)
title('Andamento giunto 2')
xlabel('Tempo [s]')
ylabel('Angolo [rad]')
grid on
grid minor

figure(4)
%subplot(4,1,3)
% plot(T,rad2deg(joints(:,3)),'-b','Linewidth',4)
plot(T,joints(:,3),'-b','Linewidth',4)
title('Andamento giunto 3')
xlabel('Tempo [s]')
ylabel('Angolo [rad]')
grid on
grid minor

figure(6)
%subplot(4,1,4)
%plot(T,rad2deg(joints(:,4)),'-b','Linewidth',4)
plot(T,joints(:,4),'-b','Linewidth',4)
title('Andamento giunto 4')
xlabel('Tempo [s]')
ylabel('Angolo [rad]')
grid on
grid minor



figure(2)
subplot(3,1,1)
plot(T,XY_err(:,1),'-b','Linewidth',4)
title('Errore posa - asse x')
xlabel('Tempo [s]')
ylabel('Errore [cm]')
grid on
grid minor


subplot(3,1,2)
plot(T,XY_err(:,2),'-b','Linewidth',4)
title('Errore posa - asse y')
xlabel('Tempo [s]')
ylabel('Errore [cm]')
grid on
grid minor


subplot(3,1,3)
plot(T,XY_err(:,3),'-b','Linewidth',4)
title('Errore posa - orientamento')
xlabel('Tempo [s]')
ylabel('Errore [rad]')
grid on
grid minor



MAKE_VIDEO = 1;
if(MAKE_VIDEO)
    motion = VideoWriter(['mov_2D_',datestr(now,30),'.avi']);
    open(motion);
end


figure(5)
for i=1:2:size(XY_IK1,1)

    plot(XY(:,1),XY(:,2),'-k','Linewidth',4)
    hold on
    plot([0 XY_IK1(i,1)],[0 XY_IK1(i,2)],'-m','Linewidth',4)
    plot([XY_IK1(i,1) XY_IK2(i,1)],[XY_IK1(i,2) XY_IK2(i,2)],'-b','Linewidth',4)
    plot([XY_IK2(i,1) XY_IK3(i,1)],[XY_IK2(i,2) XY_IK3(i,2)],'-g','Linewidth',4)
    plot([XY_IK3(i,1) XY_IK4(i,1)],[XY_IK3(i,2) XY_IK4(i,2)],'-r','Linewidth',4)
    
    axis equal
    xlim([-5 45])
    ylim([-15 37])
  

    
    
    if(MAKE_VIDEO)
        F = getframe(gcf);
        writeVideo(motion,F);
    end
    pause(0.1)
    hold off
    
end



if(MAKE_VIDEO)
    close(motion);
end

