%% Calcolo del secondo funzionale di costo ( centro giunti)
function dw2 = funzionale_dw2(Q,joint_lim)

    %Centri corsa
    q1_m= (abs(joint_lim(1,2))- abs(joint_lim(1,1)))/2;
    q2_m= (abs(joint_lim(2,2))-abs(joint_lim(2,1)))/2;
    q3_m= (abs(joint_lim(3,2))-abs(joint_lim(3,1)))/2;
    q4_m= (abs(joint_lim(4,2))-abs(joint_lim(4,1)))/2;

    %Range di giunto

    q1_range= abs(joint_lim(1,2))+ abs(joint_lim(1,1));
    q2_range= abs(joint_lim(2,2))+ abs(joint_lim(2,1));
    q3_range= abs(joint_lim(3,2))+ abs(joint_lim(3,1));
    q4_range= abs(joint_lim(4,2))+ abs(joint_lim(4,1));




 dw2=[ -(Q(1) - q1_m)/(4*q1_range^2);
     -(Q(2) - q2_m)/(4*q2_range^2);
     -(Q(3) - q3_m)/(4*q3_range^2);
     -(Q(4) - q4_m)/(4*q4_range^2)];

end