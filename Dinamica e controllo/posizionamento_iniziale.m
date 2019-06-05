function [xd,xdd,phi,phit] = posizionamento_iniziale(Qdes,Q0,t)% Q0 è la posizione iniziale che è quella allineata
    p_finale=[35, 25];
    p_iniziale=[53, 0];
    ti=0;
    tf=5;
     A=[ti^3 ti^2 ti 1;
       tf^3 tf^2 tf 1;
       3*ti^2 2*ti 1 0;
       3*tf^2 2*tf 1 0];
   
   B= [0 ;norm(p_finale-p_iniziale); 0; 0];
   
   a = A \ B;
   
   s= a(1) * t^3 + a(2) * t^2 + a(3) * t + a(4);
   
   xd= p_iniziale + s*(p_finale-p_iniziale)/B(2); 
   
   
   
   
   
   
end