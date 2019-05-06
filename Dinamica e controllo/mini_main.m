%% MIni_main
clear all
close all
clc



% link=[0.15, 0.16, 0.15 ,0.07];
% [JL,JM,Inerzia_link,Inerzia_motori,masse_link,masse_motori,Q,Q_dot,g0]= Inizializ_dinamica(link);
% [B,C,G]= Calcolo_matrici_dinamica_4DoF(JL,JM,Inerzia_link,Inerzia_motori,masse_link,masse_motori,Q,Q_dot,g0);
% load('joints.mat');
% 
 prompt = 'Che simulazione vuoi vedere? [D]ynamic_model - [P]IDcontroller - [S]imulation: ';
 str = input(prompt,'s');

if strip(lower(str)) == "d" %codice simulink
    test_dynamic_model();

elseif strip(lower(str)) == "p"
    test_PIDcontroller(joints);
else
    Q_=simulation_4dof();
end




