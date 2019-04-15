%% Algoritmo per l'inversione cinematica con la pseudo-inversa
% in uscita da questo algoritmo ho la Qddot ossia la velocità del giunto 

function Qddot = inv_cin_psd(Q,XYd,XYddot,a)
     [joint_lim,~]= inizializza_limiti();

     % calcolo la posa. Successivamente valuto l'errore ossia la differenza
     % tra la posa calcolata con la function cinematica diretta e quella
     % desiderata che otteniamo dalla traiettoria.
     [XY1 XY2 XY3 XY4]=kin_man_rid_progetto(Q,a);
     % dove XY1 è un vettore 3x1 che contiene posizione x, y e orientamento
     % del giunto 1, XY2 del giubnto 2 e cosi via. Quindi XY4 contiene
     % posizione e orientamento dell'organo terminale. 

     %errore
     e=XYd-XY4;
     Ja = J_man_plan_4DoF(Q,a);
     
     % calcolo della pseudo inversa
        J_pi=(Ja')*inv(Ja*(Ja'));
        P=eye(4)-J_pi*Ja;
             
        K=[1 0 0 ;
           0 1 0;
           0 0 1 ]*10;
                    
        if(det(Ja*Ja') == 0) % quindi mi trovo in singolarità cinematica     
            Ka=[1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
            dw1 = funzionale_dw1(Q,a);
            Qddot=J_pi*(XYddot+K*e)+P*Ka*dw1;% dove Xddot è la velocità desiderata dell'organo terminale
            
        elseif( joint_lim(1,1)+deg2rad(5) > Q(1) || Q(1) > joint_lim(1,2)-deg2rad(5)...
                    || joint_lim(2,1)+deg2rad(5) > Q(2) || Q(2) > joint_lim(2,2)-deg2rad(5)...
                    || joint_lim(3,1)+deg2rad(5) > Q(3) || Q(3) > joint_lim(3,2)-deg2rad(5)...
                    || joint_lim(4,1)+deg2rad(5) > Q(4) || Q(4) > joint_lim(4,2)-deg2rad(5)) % quindi
                % se uno qualsiasi dei giunti si trova vicino al limite di giunto
             Ka_2=[1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];    %20 10 1 130
             dw2 = funzionale_dw2(Q,joint_lim);
             Qddot=J_pi*(XYddot+K*e)+P*Ka_2*dw2; 
             
        else           
            Qddot=J_pi*(XYddot+K*e);
        end
    
    end