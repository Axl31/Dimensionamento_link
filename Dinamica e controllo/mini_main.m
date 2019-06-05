%% MIni_main
clear all
close all
clc


%% PRIMA PARTE
% queste prime tre righe servono per calcolare le matrici B,C e G per
% stamparle a video e copiarle su dynamic_model_4dof.

% link=[15, 16, 15 ,7];
% [JL,JM,Inerzia_link,Inerzia_motori,masse_link,masse_motori,Q,Q_dot,g0]= Inizializ_dinamica(link);
% [B,C,G]= Calcolo_matrici_dinamica_4DoF(JL,JM,Inerzia_link,Inerzia_motori,masse_link,masse_motori,Q,Q_dot,g0);

%% SECONDA PARTE



 prompt = 'Che simulazione vuoi vedere? [D]ynamic_model - [P]IDcontroller - [S]imulation: ';
 str = input(prompt,'s');

if strip(lower(str)) == "d" 
    % Simulazione effetto gravitazionale lungo l'asse y sul manipolatore quando è fermo 
    test_dynamic_model(); 

elseif strip(lower(str)) == "p"
    % simulazione del posizionamento
    test_PIDcontroller();
else
    % simulazione dell'esecuzione della traiettoria
    Q_=simulation_4dof();
end




