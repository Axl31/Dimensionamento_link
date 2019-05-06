function Q2dot =dynamic_model_4dof(tau,Q, Qdot)
q1=Q(1);
q2=Q(2);
q3=Q(3);
q4=Q(4);

dot_q1=Qdot(1);
dot_q2=Qdot(2);
dot_q3=Qdot(3);
dot_q4=Qdot(4);

%va cambiata perch� vanno cambiate le maase dei link, dei motori etc                                                
B= [ (63*cos(q2 + q3 + q4))/5000000 + (39*cos(q2 + q3))/40000 + (21*cos(q3 + q4))/1562500 + (5817*cos(q2))/1000000 + (13*cos(q3))/12500 + (63*cos(q4))/5000000 + 57920919875383847493792642629/5764607523034234880000000000000, (63*cos(q2 + q3 + q4))/10000000 + (39*cos(q2 + q3))/80000 + (21*cos(q3 + q4))/1562500 + (5817*cos(q2))/2000000 + (13*cos(q3))/12500 + (63*cos(q4))/5000000 + 35977828895672609769851233/9223372036854775808000000000, (63*cos(q2 + q3 + q4))/10000000 + (39*cos(q2 + q3))/80000 + (21*cos(q3 + q4))/3125000 + (13*cos(q3))/25000 + (63*cos(q4))/5000000 + 132641240627953744326895501/202914184810805067776000000000, (63*cos(q2 + q3 + q4))/10000000 + (21*cos(q3 + q4))/3125000 + (63*cos(q4))/10000000 + 1193353308851904432160513/101457092405402533888000000000;
      (63*cos(q2 + q3 + q4))/10000000 + (39*cos(q2 + q3))/80000 + (21*cos(q3 + q4))/1562500 + (5817*cos(q2))/2000000 + (13*cos(q3))/12500 + (63*cos(q4))/5000000 + 35977828895672609769851233/9223372036854775808000000000,                                                                                      (21*cos(q3 + q4))/1562500 + (13*cos(q3))/12500 + (63*cos(q4))/5000000 + 53436375355330318593846233/9223372036854775808000000000,                                        (21*cos(q3 + q4))/3125000 + (63*cos(q2))/62500 + (13*cos(q3))/25000 + (63*cos(q4))/5000000 + 324395145274164533375215501/202914184810805067776000000000,                                   (21*cos(q3 + q4))/3125000 + (63*cos(q4))/10000000 + 1193353308851904432160513/101457092405402533888000000000;
                            (63*cos(q2 + q3 + q4))/10000000 + (39*cos(q2 + q3))/80000 + (21*cos(q3 + q4))/3125000 + (13*cos(q3))/25000 + (63*cos(q4))/5000000 + 132641240627953744326895501/202914184810805067776000000000,                                                              (21*cos(q3 + q4))/3125000 + (63*cos(q2))/62500 + (13*cos(q3))/25000 + (63*cos(q4))/5000000 + 324395145274164533375215501/202914184810805067776000000000,                                                                                       (63*cos(q2))/31250 + (63*cos(q4))/5000000 + 5969867028039492925051896461/2232056032918855745536000000000,                                                               (63*cos(q4))/10000000 + 1193353308851904432160513/101457092405402533888000000000;
                                                                            (63*cos(q2 + q3 + q4))/10000000 + (21*cos(q3 + q4))/3125000 + (63*cos(q4))/10000000 + 1193353308851904432160513/101457092405402533888000000000,                                                                                                         (21*cos(q3 + q4))/3125000 + (63*cos(q4))/10000000 + 1193353308851904432160513/101457092405402533888000000000,                                                                                                               (63*cos(q4))/10000000 + 1193353308851904432160513/101457092405402533888000000000,                                                                                       6966889056017814457937509/558014008229713936384000000000];
                                                   
C =[ - dot_q2*((63*sin(q2 + q3 + q4))/10000000 + (39*sin(q2 + q3))/80000 + (5817*sin(q2))/2000000) - dot_q4*((63*sin(q2 + q3 + q4))/10000000 + (21*sin(q3 + q4))/3125000 + (63*sin(q4))/10000000) - dot_q3*((63*sin(q2 + q3 + q4))/10000000 + (39*sin(q2 + q3))/80000 + (21*sin(q3 + q4))/3125000 + (13*sin(q3))/25000), - dot_q4*((63*sin(q2 + q3 + q4))/10000000 + (21*sin(q3 + q4))/3125000 + (63*sin(q4))/10000000) - dot_q3*((63*sin(q2 + q3 + q4))/10000000 + (39*sin(q2 + q3))/80000 + (21*sin(q3 + q4))/3125000 + (13*sin(q3))/25000) - (3*(dot_q1 + dot_q2)*(21*sin(q2 + q3 + q4) + 1625*sin(q2 + q3) + 9695*sin(q2)))/10000000, - dot_q4*((63*sin(q2 + q3 + q4))/10000000 + (21*sin(q3 + q4))/3125000 + (63*sin(q4))/10000000) - ((dot_q1 + dot_q2 + dot_q3)*(315*sin(q2 + q3 + q4) + 24375*sin(q2 + q3) + 336*sin(q3 + q4) + 26000*sin(q3)))/50000000, -(21*(15*sin(q2 + q3 + q4) + 16*sin(q3 + q4) + 15*sin(q4))*(dot_q1 + dot_q2 + dot_q3 + dot_q4))/50000000;
                                                                                                 dot_q1*((63*sin(q2 + q3 + q4))/10000000 + (39*sin(q2 + q3))/80000 + (5817*sin(q2))/2000000) - dot_q3*((21*sin(q3 + q4))/3125000 + (13*sin(q3))/25000) - dot_q4*((21*sin(q3 + q4))/3125000 + (63*sin(q4))/10000000),                                                                                                                                                                                          - dot_q3*((21*sin(q3 + q4))/3125000 + (13*sin(q3))/25000) - dot_q4*((21*sin(q3 + q4))/3125000 + (63*sin(q4))/10000000),             - ((dot_q1 + dot_q2)*(21*sin(q3 + q4) + 1625*sin(q3)))/3125000 - dot_q4*((21*sin(q3 + q4))/3125000 + (63*sin(q4))/10000000) - dot_q3*((21*sin(q3 + q4))/3125000 - (63*sin(q2))/62500 + (13*sin(q3))/25000),                        -(21*(16*sin(q3 + q4) + 15*sin(q4))*(dot_q1 + dot_q2 + dot_q3 + dot_q4))/50000000;
                                                                                                       dot_q1*((63*sin(q2 + q3 + q4))/10000000 + (39*sin(q2 + q3))/80000 + (21*sin(q3 + q4))/3125000 + (13*sin(q3))/25000) + dot_q2*((21*sin(q3 + q4))/3125000 + (13*sin(q3))/25000) - (63*dot_q4*sin(q4))/10000000,                                                                                                               dot_q1*((21*sin(q3 + q4))/3125000 + (13*sin(q3))/25000) - (63*dot_q3*sin(q2))/62500 - (63*dot_q4*sin(q4))/10000000 + dot_q2*((21*sin(q3 + q4))/3125000 - (63*sin(q2))/62500 + (13*sin(q3))/25000),                                                                                                                                                             - (63*dot_q2*sin(q2))/62500 - (63*dot_q4*sin(q4))/10000000,                                               -(63*sin(q4)*(dot_q1 + dot_q2 + dot_q3 + dot_q4))/10000000;
                                                                                                                           dot_q1*((63*sin(q2 + q3 + q4))/10000000 + (21*sin(q3 + q4))/3125000 + (63*sin(q4))/10000000) + dot_q2*((21*sin(q3 + q4))/3125000 + (63*sin(q4))/10000000) + (63*dot_q3*sin(q4))/10000000,                                                                                                                                                                                                                   (21*(dot_q1 + dot_q2)*(16*sin(q3 + q4) + 15*sin(q4)))/50000000 + (63*dot_q3*sin(q4))/10000000,                                                                                                                                                                       (63*sin(q4)*(dot_q1 + dot_q2 + dot_q3))/10000000,                                                                                                        0];
 
G =[0;0;0;0];                         
                  
Fv=eye(4)*1e-1;
                          
% eq. della dinamica diretta                        
Q2dot=B\(tau-C*Qdot-Fv*sign(Qdot)+G);

end