function Q2dot =dynamic_model_4dof_pendolo(tau,Q, Qdot)
% Q2dot =dynamic_model_4dof_pendolo(tau,Q, Qdot)
% funzione all'interno della quale sono riportate e vengono valutate le matrici B,C e G
% ottenute dalla funzione Calcolo_matrici_dinamica_4DoF, nella quale si
% calcola la dinamica diretta. 
q1=Q(1);
q2=Q(2);
q3=Q(3);
q4=Q(4);

dot_q1=Qdot(1);
dot_q2=Qdot(2);
dot_q3=Qdot(3);
dot_q4=Qdot(4);

% Matrice di inerzia                                               
B=  [ 126*cos(q2 + q3 + q4) + 9750*cos(q2 + q3) + (672*cos(q3 + q4))/5 + 58170*cos(q2) + 10400*cos(q3) + 126*cos(q4) + 50238389729/500000, 63*cos(q2 + q3 + q4) + 4875*cos(q2 + q3) + (672*cos(q3 + q4))/5 + 29085*cos(q2) + 10400*cos(q3) + 126*cos(q4) + 156028961/4000, 63*cos(q2 + q3 + q4) + 4875*cos(q2 + q3) + (336*cos(q3 + q4))/5 + 5200*cos(q3) + 126*cos(q4) + 115047937/17600, 63*cos(q2 + q3 + q4) + (336*cos(q3 + q4))/5 + 63*cos(q4) + 1035069/8800;
     63*cos(q2 + q3 + q4) + 4875*cos(q2 + q3) + (672*cos(q3 + q4))/5 + 29085*cos(q2) + 10400*cos(q3) + 126*cos(q4) + 156028961/4000,                                                            (672*cos(q3 + q4))/5 + 10400*cos(q3) + 126*cos(q4) + 231743337/4000,                            (336*cos(q3 + q4))/5 + 10080*cos(q2) + 5200*cos(q3) + 126*cos(q4) + 281367937/17600,                        (336*cos(q3 + q4))/5 + 63*cos(q4) + 1035069/8800;
                      63*cos(q2 + q3 + q4) + 4875*cos(q2 + q3) + (336*cos(q3 + q4))/5 + 5200*cos(q3) + 126*cos(q4) + 115047937/17600,                                            (336*cos(q3 + q4))/5 + 10080*cos(q2) + 5200*cos(q3) + 126*cos(q4) + 281367937/17600,                                                                20160*cos(q2) + 126*cos(q4) + 5178034241/193600,                                               63*cos(q4) + 1035069/8800;
                                                             63*cos(q2 + q3 + q4) + (336*cos(q3 + q4))/5 + 63*cos(q4) + 1035069/8800,                                                                               (336*cos(q3 + q4))/5 + 63*cos(q4) + 1035069/8800,                                                                                      63*cos(q4) + 1035069/8800,                                                           6042813/48400];
 
% Matrice relativa alle forze centrifughe e di Coriolis                                                                      
C = [ - dot_q4*(63*sin(q2 + q3 + q4) + (336*sin(q3 + q4))/5 + 63*sin(q4)) - dot_q2*(63*sin(q2 + q3 + q4) + 4875*sin(q2 + q3) + 29085*sin(q2)) - dot_q3*(63*sin(q2 + q3 + q4) + 4875*sin(q2 + q3) + (336*sin(q3 + q4))/5 + 5200*sin(q3)), - dot_q4*(63*sin(q2 + q3 + q4) + (336*sin(q3 + q4))/5 + 63*sin(q4)) - dot_q3*(63*sin(q2 + q3 + q4) + 4875*sin(q2 + q3) + (336*sin(q3 + q4))/5 + 5200*sin(q3)) - 3*(dot_q1 + dot_q2)*(21*sin(q2 + q3 + q4) + 1625*sin(q2 + q3) + 9695*sin(q2)), - dot_q4*(63*sin(q2 + q3 + q4) + (336*sin(q3 + q4))/5 + 63*sin(q4)) - ((dot_q1 + dot_q2 + dot_q3)*(315*sin(q2 + q3 + q4) + 24375*sin(q2 + q3) + 336*sin(q3 + q4) + 26000*sin(q3)))/5, -(21*(15*sin(q2 + q3 + q4) + 16*sin(q3 + q4) + 15*sin(q4))*(dot_q1 + dot_q2 + dot_q3 + dot_q4))/5;
                                                                    dot_q1*(63*sin(q2 + q3 + q4) + 4875*sin(q2 + q3) + 29085*sin(q2)) - dot_q4*((336*sin(q3 + q4))/5 + 63*sin(q4)) - dot_q3*((336*sin(q3 + q4))/5 + 5200*sin(q3)),                                                                                                                                                   - dot_q4*((336*sin(q3 + q4))/5 + 63*sin(q4)) - dot_q3*((336*sin(q3 + q4))/5 + 5200*sin(q3)),              - (16*(dot_q1 + dot_q2)*(21*sin(q3 + q4) + 1625*sin(q3)))/5 - dot_q4*((336*sin(q3 + q4))/5 + 63*sin(q4)) - dot_q3*((336*sin(q3 + q4))/5 - 10080*sin(q2) + 5200*sin(q3)),                        -(21*(16*sin(q3 + q4) + 15*sin(q4))*(dot_q1 + dot_q2 + dot_q3 + dot_q4))/5;
                                                                        dot_q1*(63*sin(q2 + q3 + q4) + 4875*sin(q2 + q3) + (336*sin(q3 + q4))/5 + 5200*sin(q3)) + dot_q2*((336*sin(q3 + q4))/5 + 5200*sin(q3)) - 63*dot_q4*sin(q4),                                                                                        dot_q1*((336*sin(q3 + q4))/5 + 5200*sin(q3)) - 10080*dot_q3*sin(q2) - 63*dot_q4*sin(q4) + dot_q2*((336*sin(q3 + q4))/5 - 10080*sin(q2) + 5200*sin(q3)),                                                                                                                                           - 10080*dot_q2*sin(q2) - 63*dot_q4*sin(q4),                                                   -63*sin(q4)*(dot_q1 + dot_q2 + dot_q3 + dot_q4);
                                                                                                dot_q1*(63*sin(q2 + q3 + q4) + (336*sin(q3 + q4))/5 + 63*sin(q4)) + dot_q2*((336*sin(q3 + q4))/5 + 63*sin(q4)) + 63*dot_q3*sin(q4),                                                                                                                                                                   (21*(dot_q1 + dot_q2)*(16*sin(q3 + q4) + 15*sin(q4)))/5 + 63*dot_q3*sin(q4),                                                                                                                                                63*sin(q4)*(dot_q1 + dot_q2 + dot_q3),                                                                                                 0];
% G è diversa da 0 perché abbiamo imposto la gravità sull'asse y
G = [ - (12753*cos(q1 + q2 + q3))/4 - (20601*cos(q1 + q2 + q3 + q4))/500 - (1902159*cos(q1 + q2))/100 - (47290666736328470949*cos(q1))/1099511627776000;
                                 (61803*cos(q1))/5 - (20601*cos(q1 + q2 + q3 + q4))/500 - (1902159*cos(q1 + q2))/100 - (12753*cos(q1 + q2 + q3))/4;
                                  (164808*cos(q1 + q2))/25 - (20601*cos(q1 + q2 + q3 + q4))/500 - (12753*cos(q1 + q2 + q3))/4 + (61803*cos(q1))/10;
                                                                                                               -(20601*cos(q1 + q2 + q3 + q4))/500];

                           
% Matrice attrito statico                  
Fs=eye(4)*1e-1;
                          
% eq. della dinamica diretta                        
Q2dot=B\(tau-C*Qdot-Fs*sign(Qdot)+G);

end