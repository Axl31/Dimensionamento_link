%% Calcolo del primo funzionale di costo( misura di manipolabilità)
% dw1 = funzionale_dw1(Q,a)
% Calcolo del primo funzionale di costo(misura di manipolabilità)
% dw1 = sqrt(det(J*J')) 
function dw1 = funzionale_dw1(Q,a)

    q1=Q(1);
    q2=Q(2);
    q3=Q(3);
    q4=Q(4);
    
    a1=a(1);
    a2=a(2);
    a3=a(3);
    a4=a(4);
 
    dw1= [ 0 ;
           (a1*(a2*a3^2*sin(q2 + 2*q3) - a2*a3^2*sin(q2) + 2*a1*a2^2*sin(2*q2)...
           + 2*a1*a3^2*sin(2*q2 + 2*q3) + 2*a1*a2*a3*sin(2*q2 + q3)))/(2*conj((a1^2*a2^2 +...
           a1^2*a3^2 + a2^2*a3^2 - a1^2*a2^2*cos(2*q2) - a2^2*a3^2*cos(2*q3) - ...
           a1^2*a3^2*cos(2*q2 + 2*q3) + a1*a2*a3^2*cos(q2) + a1^2*a2*a3*cos(q3) - ...
           a1*a2*a3^2*cos(q2 + 2*q3) - a1^2*a2*a3*cos(2*q2 + q3))^(1/2)));
          (a3*(a1^2*a2*sin(2*q2 + q3) - a1^2*a2*sin(q3) + 2*a2^2*a3*sin(2*q3) +...
          2*a1^2*a3*sin(2*q2 + 2*q3) + 2*a1*a2*a3*sin(q2 + 2*q3)))/(2*conj((a1^2*a2^2 +...
          a1^2*a3^2 + a2^2*a3^2 - a1^2*a2^2*cos(2*q2) - a2^2*a3^2*cos(2*q3) - ...
          a1^2*a3^2*cos(2*q2 + 2*q3) + a1*a2*a3^2*cos(q2) + a1^2*a2*a3*cos(q3) -...
          a1*a2*a3^2*cos(q2 + 2*q3) - a1^2*a2*a3*cos(2*q2 + q3))^(1/2)));
          0];
                                                                                                                                                                                                                                                                                                                                                             
end