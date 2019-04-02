function [joint_lim, link_lim] = inizializza_limiti()
   %FUNZIONE UTILIZZATA PER IL MAIN PER OTTIMIZZARE I LINK   
%Limiti di giunto del manipolatore
    joint_lim=[deg2rad(-70) deg2rad(70);
        deg2rad(0) deg2rad(140);
        deg2rad(0) deg2rad(140);
        deg2rad(-100) deg2rad(100)];
    
    %Grandezza minima/massima dei link
    link_lim=[15 20; 
        10 18;
        8 18; 
        7 18];

end