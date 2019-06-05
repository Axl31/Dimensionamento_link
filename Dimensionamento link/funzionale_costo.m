function dw = funzionale_costo(joint_lim,Q,a)
    %% PRIMO FUNZIONALE DI COSTO (misura di manipolabilità)
    % w = sqrt(det(J*J')) 
    
    dw1 = funzionale_dw1(Q,a);

    %% SECONDO FUNZIONALE DI COSTO (centro giunti)    
    %Il secondo funzionale è quello che cerca di mantenere i giunti lontano dai
    %fine corsa
    
    dw2 = funzionale_dw2(Q,joint_lim);
       
    %% Sommo ora i 2 funzionali
    dw=dw1+dw2;

end
