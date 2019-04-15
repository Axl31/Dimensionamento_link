function [xd,xdd,phi,phit]=planner_TOAD(punto, tempo, phi, t, circonferenza1, circonferenza2)
    %FUNZIONE UTILIZZATA PER IL MAIN PER OTTIMIZZARE I LINK
    %mi definisco i parametri da utilizzare per la pianificazione della
    %traiettoria e dell'orientamento:
    %pf=punto.finale; %punto finale per il primo tratto, coincide con il punto A
    punto_iniziale=punto.iniziale; %punto iniziale per il primo tratto, coincide con il punto B
    ti=tempo.iniziale; %tempo iniziale
    tf=tempo.finale1; %tempo finale per il primo tratto
    tf2=tempo.finale2; %tempo finale per il secondo tratto
    phi_i=phi.iniziale; %phi iniziale per il primo tratto, ovvero in B
    phi_f=phi.finale; %phi finale per il primo tratto, ovvero in A
    c1 = circonferenza1.centro;
    r1 = circonferenza1.raggio;
    c2 = circonferenza2.centro;
    r2 = circonferenza2.raggio;

    alfa_2=deg2rad(180); %angolo di rotazione per il primo tratto
    beta=deg2rad(268.84);

        
    
    

    if t<=ti %se ci troviamo ad un tempo inferiore a quello iniziale allora ci troviamo nella
            %configurazione iniziale sia per la posizione che per l'orientamento
            XYd = punto_iniziale;
            XYddot = [0, 0];

    elseif t <= tf %se ci troviamo ad un tempo inferiore di tf, dobbiamo pianificare
        %la traiettoria circolare del primo tratto, con raggio 'r1' e centro
        %'c1'
        A_1=[ti^3 ti^2 ti 1;
           tf^3 tf^2 tf 1;
           3*ti^2 2*ti 1 0;
           3*tf^2 2*tf 1 0]; %definisco la matrice dei termini noti temporali, derivante
        %dalle condizioni di definizione della legge oraria 's(t)', essa vale sia
        %per l'orientamento che per la traiettoria
        B_1 = [0 ;r1 * alfa_2; 0; 0]; %vettore dei termini noti per la traiettoria
        a_1 = A_1 \ B_1; %coefficienti della legge oraria per la traiettoria

        s= a_1(1) * t^3 + a_1(2) * t^2 + a_1(3) * t + a_1(4); %legge oraria della traiettoria
        %sdot=3*a_2(1)*t^2+2*a_2(2)*t+a_2(3); %derivata della legge oraria

        th = deg2rad(0);

        R1=[cos(th) -sin(th);
            sin(th) cos(th)];
        %rotazione per la pianificazione di una traiettoria
        %circolare, sappiamo che l'equazione si riferisce ad una terna concentrica
        %a 'c1' e con angolo nullo quando ci troviamo nella situazione s(0)



        XYd = (c1' + R1 * [r1*cos(s/r1); r1*sin(s/r1)])'; %è la x desiderata
        XYddot = (R1 * [-sin(s/r1); cos(s/r1)])';


    elseif t > tf2%altrimenti rimaniamo alla configurazione da cui siamo partiti ovvero nel punto B
        XYd=punto_iniziale;
        XYddot=[0, 0];

    else %se ci troviamo ad un tempo maggiore di tf siamo passati a descrivere
        %la seconda traiettoria:
        A_2 = [tf^3 tf^2 tf 1;
           tf2^3 tf2^2 tf2 1;
           3*tf^2 2*tf 1 0;
           3*tf2^2 2*tf2 1 0]; %definisco come prima la matrice A
        B_2 = [0; r2 * beta; 0; 0]; %vettore termini noti per la traiettoria 
        a_2= A_2 \ B_2; %coefficienti della legge oraria della traiettoria

        s= a_2(1) * t^3 + a_2(2) * t^2 + a_2(3) * t + a_2(4); %legge oraria traiettoria
        %sdot=3*a_1(1)*t^2+2*a_1(2)*t+a_1(3); %derivata legge oraria

        gamma=deg2rad(135.58);

        R2=[cos(gamma) -sin(gamma);
            sin(gamma) cos(gamma)]; %& pi %matrice di rotazione per rimetterci nelle condizioni
        %per la pianificazione della traiettoria con centro 'c2' e raggio 'r2'



        
        XYd = (c2' + R2 * [r2*cos(s/r2); r2*sin(s/r2)])'; %x desiderata
        XYddot = (R2 * [-sin(s/r2); cos(s/r2)])';



    end 
    

%definisco l'orientamento secondo una legge oraria lineare, ricordando che
%in questo caso il tempo iniziale è ti e il tempo finale tf2, pertanto si
%faccia attenzione alla matrice A che risulta diversa dai casi precedenti.
A=[ti^3 ti^2 ti 1;
   tf2^3 tf2^2 tf2 1;
   3*ti^2 2*ti 1 0;
   3*tf2^2 2*tf2 1 0];
B=[0;pi/6;0;0];
a=inv(A)*B;

%Calcolo di s e sdot (a partire dai parametri determinati)
s=a(1)*t^3+a(2)*t^2+a(3)*t+a(4);
sdot=3*a(1)*t^2+2*a(2)*t+a(3);

% Prima del tempo iniziale mantiene posizione iniziale e velocità nulla
% Dopo del tempo finale mantiene posizione finale e velocità nulla
if t<ti
    phi_e=0;
    phi_et=0;
else
    if t<=tf2
    phi_e=phi_i+(s/norm(pi/6))*(phi_f-phi_i);
    phi_et=(sdot/norm(pi/6))*(phi_f-phi_i);
    else
    phi_e=phi_f;
    phi_et=0;
    end
end
% Posizioni e velocità desiderate


    %salvo i dati che voglio in uscita
    xd=XYd;
    xdd=XYddot;
    phi=phi_e;
    phit=phi_et;
 
end