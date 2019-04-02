function min_q4 = prova(link, XY)
a1=link(1);
a2=link(2);
a3=link(3);
a4=link(4);

[joint_lim, ~] = inizializza_limiti();

tf=15; %definisco il tempo finale
dt=tf/size(XY,1); %definisco il passo temporale
%T=(0:dt:tf-dt); %definisco il vettore dei tempi

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


 min_q4= min(joints(:,4));



end