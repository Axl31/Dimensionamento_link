% clear all
% close all
% clc

function [] = test_dynamic_model()
%% Variables initialization
a1=15;
a2=16;
a3=15;
a4=7;

T(1)=0.0;
time=0.0;
i=2;
start_time=tic;% prende il valore temporale nell'istante di tempo in cui viene eseguita la riga di codice

Q=[pi/6;pi/6;pi/6;pi/6];%0;0;0;0
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

%% Simulation loop
while(time<10)  
time=toc(start_time);   % tempo corrente. in poche parole prendo il tempo che trascorre nell'esecuzione 
% tra il comando tic e toc
T(i)=time; 
dt=T(i)-T(i-1);

%% Robot Dynamic Model
Q2dot=dynamic_model_4dof(tau,Q, Qdot); % ottengo l'accelerazione dalla quale integrando ricavo velocità e posizione
Qdot=Qdot+Q2dot*dt;
Q=Q+Qdot*dt;

%% Variables saving
Q2dot_=[Q2dot_;Q2dot'];
Qdot_=[Qdot_;Qdot'];
Q_=[Q_;Q'];

[xy1, xy2, xy3, xy4]=kin_man_rid_progetto(Q,[a1,a2,a3,a4]);
    
    XY1=[XY1; xy1'];
    XY2=[XY2; xy2'];
    XY3=[XY3; xy3'];
    XY4=[XY4; xy4'];

i=i+1;
end




%% Video
MAKE_VIDEO = 1;
if(MAKE_VIDEO)
    motion = VideoWriter(['mov_2D_',datestr(now,30),'.avi']);
    open(motion);
end


figure(1)
title('Simulazione robot soggetto a gravità')
for i=1:40:size(XY1,1)
    axis equal      
    plot([0 XY1(i,1)],[0 XY1(i,2)],'-r','Linewidth',4)
    hold on
    plot([XY1(i,1) XY2(i,1)],[XY1(i,2) XY2(i,2)],'-b','Linewidth',4)
    plot([XY2(i,1) XY3(i,1)],[XY2(i,2) XY3(i,2)],'-g','Linewidth',4)
    plot([XY3(i,1) XY4(i,1)],[XY3(i,2) XY4(i,2)],'-k','Linewidth',4)
    axis equal
    
    if(MAKE_VIDEO)
        F = getframe(gcf);
        writeVideo(motion,F);
    end
    %pause(0.5)
    hold off
    
end



if(MAKE_VIDEO)
    close(motion);
end
end
