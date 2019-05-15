%% Definizione dei parametri
function [JL,JM,Inerzia_link,Inerzia_motori,masse_link,masse_motori,Q,Q_dot,g0]= Inizializ_dinamica(link)

syms q1 q2 q3 q4 dot_q1 dot_q2 dot_q3 dot_q4 'real'

%% PARAMETRI DINAMICI DEL ROBOT - valori numerici
%Lunghezze link [m]
l1=link(1);
l2=link(2);
l3=link(3);
l4=link(4);

%Sezione link [m]
% % Si assume che i link siano parallelepipedi
% a=0.05;
% b=0.02;
% i nostri link non sono parallepipedi. Abbiamo simulato la parte dello
% statore attaccata al link insieme al supporto del moto riduttore e le
% viti. L'inerzia l'abbiamo calcolata direttamente con onshape. Abbiamo
% fissato il centro di rotazione di onshape con quello del nostro link.

%Rotore motori
% Si assume che il rotore del motore sia un cilindro 
%di altezza h e raggio r
h_m=0.01;%0.02;
r_m=0.0115/2;%0.015;

h_M=0.024;
r_M=0.0116/2;

%Masse link [kg]

ml_1=0.030;
ml_2=0.030;
ml_3=0.030;
ml_4=0.014;

%Masse statori motori [kg]
mr_1=0.107;
mr_2=0.107;
mr_3=0.070;
mr_4=0.070;

%Masse totali Link [kg]
M_1=ml_1+mr_2;
M_2=ml_2+mr_3;
M_3=ml_3+mr_4;
M_4=ml_4;
masse_link = [M_1, M_2, M_3, M_4];

%Masse motori [kg]
M_m1=0.085;
M_m2=0.085;
M_m3=0.042;
M_m4=0.042;
masse_motori = [M_m1, M_m2, M_m3, M_m4];


%Coefficienti di riduzione motori 
kr1=2;
kr2=2;
kr3=36/22;
kr4=36/22;
  

%% Centri di massa dei link 

COM_1=[0.08572-l1; 0; 0];     %Terna1
COM_2=[0.0115-l2; 0; 0];      %Terna2
COM_3=[0.0115-l3; 0; 0];     %Terna3
COM_4=[0.003-l4; 0; 0];

%Centri di massa dei Motori/Rotori rispetto alla terna precedente
COM_m1=[0; 0; 0];           %Terna 0
COM_m2=[0; 0; 0];           %Terna 1
COM_m3=[0; 0; 0];           %Terna 2
COM_m4=[0; 0; 0];           %Terna 3


%% TENSORI INERZIA LINK [kgm^2]
%Link 1 rispetto Terna 1

% I_1=[1/12*M_1*(a^2+b^2)          0                   0;
%             0           1/12*M_1*(b^2+l1^2)          0;
%             0                    0          1/12*M_1*(a^2+l1^2)];

I_1= diag([0,0,3.21e-4]);

% %Link 2 rispetto Terna 2
% I_2=[1/12*M_2*(a^2+b^2)          0                   0;
%             0           1/12*M_2*(b^2+l2^2)          0;
%             0                    0          1/12*M_2*(a^2+l2^2)];

I_2= diag([0,0,3.13e-4]);

% %Link 3 Terna 3
% I_3=[1/12*M_3*(a^2+b^2)          0                   0;
%             0           1/12*M_3*(b^2+l3^2)          0;
%             0                    0          1/12*M_3*(a^2+l3^2)];

I_3= diag([0,0,3.13e-4]);
        
%Link 4 Terna 4
% I_4=[1/12*M_4*(a^2+b^2)          0                   0;
%             0           1/12*M_4*(b^2+l4^2)          0;
%             0                    0          1/12*M_4*(a^2+l4^2)];
 
I_4= diag([0,0,1.05e-5]);

% Inerzia_link  = struct;
% Inerzia_link.I_1=I_1;
% Inerzia_link.I_2=I_2;
% Inerzia_link.I_3=I_3;
% Inerzia_link.I_4=I_4;
% A noi non servono queste inerzie ma ci serve I_01 etc che stanno dopo 



%% Tensori di inerzia dei Motori [kgm^2]
%Motore1 rispetto Terna 0
I_m1_b=[1/12*M_m1*(h_M^2+3*r_M^2)         0                 0;
                0             1/12*M_m1*(h_M^2+3*r_M^2)     0;
                0                     0             1/2*M_m1*r_M^2];



%Motore2 rispetto Terna 1
I_m2_b=[1/12*M_m2*(h_M^2+3*r_M^2)         0                 0;
                0             1/12*M_m2*(h_M^2+3*r_M^2)     0;
                0                     0             1/2*M_m2*r_M^2];
%Motore3 rispetto Terna 2
I_m3_b=[1/12*M_m3*(h_m^2+3*r_m^2)         0                 0;
                0             1/12*M_m3*(h_m^2+3*r_m^2)     0;
                0                     0             1/2*M_m3*r_m^2];

%Motore4 rispetto Terna 3
I_m4_b=[1/12*M_m4*(h_m^2+3*r_m^2)         0                 0;
                0             1/12*M_m4*(h_m^2+3*r_m^2)     0;
                0                     0             1/2*M_m4*r_m^2];
            
% Inerzia_motori= struct;
% Inerzia_motori.I_m1_b = I_m1_b;
% Inerzia_motori.I_m2_b = I_m2_b;
% Inerzia_motori.I_m3_b = I_m3_b;
% Inerzia_motori.I_m4_b = I_m4_b;
% Non ci servono queste inerzie dei motori ma quelle che stanno dopo I_m1 etc            

%% Vettore g
% il vettore g nel nostro caso non ci serve perchè non lavora contro
% gravità
g0=[0 -9.81 0]';

%% RISOLUZIONE SIMBOLICA CINEMATICA
Q=[q1; q2; q3; q4];
Q_dot=[dot_q1; dot_q2; dot_q3;dot_q4];

T_01=calcolo_DH(0,l1,0,q1);
T_12=calcolo_DH(0,l2,0,q2);
T_23=calcolo_DH(0,l3,0,q3);
T_34=calcolo_DH(0,l4,0,q4);
    
T_02=T_01*T_12;
T_03=T_02*T_23;
T_04=T_03*T_34;


%Orientamento asse z delle terne di riferimento
z_0=[0;0;1];
z_1=T_01(1:3,3);
z_2=T_02(1:3,3);
z_3=T_03(1:3,3);
z_4=T_04(1:3,3);

%Posizione dell'Origine delle Terne rispetto alla terna base
% mi prendo il vettore posizione dalle singole matrici di trasformazione
% omogenea
p_0=[0;0;0];
p_1=T_01(1:3,4);
p_2=T_02(1:3,4);
p_3=T_03(1:3,4);
p_e=T_04(1:3,4);

%% COMPONENTI DELLA LAGRANGIANA

%% Link 1
% Si esprime la posizione del baricentro del Link1 in terna 1
P1_=T_01*[COM_1;1];
P1=P1_(1:3);
%Tensore di inerzia espresso in terna base

Inerzia_link  = struct; %&%&%&%&%&%

I_01=T_01(1:3,1:3)*I_1*T_01(1:3,1:3)';% premoltiplico e postmoltiplico per la matrice di rotazione

Inerzia_link.I_01=I_01; %&%&%&%&%

%Jacobiano geometrico del link 1
JL1 = struct;
JP1_l1=cross(z_0,P1);
JL1.JP_l1=[JP1_l1 zeros(3,3)];

JO1_l1=z_0;
JL1.JO_l1=[JO1_l1 zeros(3,3)];

%% Link 2
P2_=T_02*[COM_2;1];
P2=P2_(1:3);
I_02=T_02(1:3,1:3)*(I_2)*T_02(1:3,1:3)';

Inerzia_link.I_02=I_02;%&%&%&%&%&%

%Jacobiano geometrico del link 2
JL2 = struct;
JP1_l2=cross(z_0,P2);
JP2_l2=cross(z_1,P2-p_1);
JL2.JP_l2=[JP1_l2 JP2_l2 zeros(3,2)];

JO1_l2=z_0;
JO2_l2=z_1;
JL2.JO_l2=[JO1_l2 JO2_l2 zeros(3,2)];

%% Link 3
P3_=T_03*[COM_3;1];
P3=P3_(1:3);
I_03=T_03(1:3,1:3)*(I_3)*T_03(1:3,1:3)';

Inerzia_link.I_03=I_03;%&%&%&%&%&%

%Jacobiano geometrico del link 3
JL3 = struct;
JP1_l3=cross(z_0,P3);
JP2_l3=cross(z_1,P3-p_1);
JP3_l3=cross(z_2,P3-p_2);
JL3.JP_l3=[JP1_l3 JP2_l3 JP3_l3 zeros(3,1)];

JO1_l3=z_0;
JO2_l3=z_1;
JO3_l3=z_2;
JL3.JO_l3=[JO1_l3 JO2_l3 JO3_l3 zeros(3,1)];

%% Link 4
P4_=T_04*[COM_4;1];
P4=P4_(1:3);
I_04=T_04(1:3,1:3)*(I_4)*T_04(1:3,1:3)';

Inerzia_link.I_04=I_04;%&%&%&%&%&%

%Jacobiano geometrico del link 3
JL4 = struct;
JP1_l4=cross(z_0,P4);
JP2_l4=cross(z_1,P4-p_1);
JP3_l4=cross(z_2,P4-p_2);
JP4_l4=cross(z_3,P4-p_3);
JL4.JP_l4=[JP1_l4 JP2_l4 JP3_l4 JP4_l4 ];

JO1_l4=z_0;
JO2_l4=z_1;
JO3_l4=z_2;
JO4_l4=z_3;
JL4.JO_l4=[JO1_l4 JO2_l4 JO3_l4 JO4_l4];


JL = struct;
JL.JL1 = JL1;
JL.JL2 = JL2;
JL.JL3 = JL3;
JL.JL4 = JL4;
%% Motori
% Motore 1. Esprimo la posizione del baricentro del motore in terna base e 
% riporto il tensore di inerzia in terna base
pm1=COM_m1;  %Già espresso in terna base

Inerzia_motori= struct;%&%&%&%&%

I_m1=I_m1_b; %È già espresso in terna base

Inerzia_motori.I_m1 = I_m1;%&%&%&%&%

JM1 = struct;
JM1.JP_m1=zeros(3,4);

JO1_m1=kr1*z_0;
JM1.JO_m1=[JO1_m1 zeros(3,3)];

% Motore 2
pm2=T_01(1:3,1:3)*COM_m2;
I_m2=T_01(1:3,1:3)*I_m2_b*T_01(1:3,1:3)';

Inerzia_motori.I_m2 = I_m2;%&%&%&%&%&%

JM2 = struct;
JP1_m2=cross(z_0,pm2-p_0);
JM2.JP_m2=[JP1_m2 zeros(3,3)];

JO2_m2=kr2*z_1;
JM2.JO_m2=[JO1_l2 JO2_m2 zeros(3,2)];

% Motore 3
pm3=T_02(1:3,1:3)*COM_m3;
I_m3=T_02(1:3,1:3)*I_m3_b*T_02(1:3,1:3)';

Inerzia_motori.I_m3 = I_m3;%&%&%&%&%&%

JM3 = struct;
JP1_m3=cross(z_0,pm3-p_0);
JP2_m3=cross(z_1,pm3-p_1);
JM3.JP_m3=[JP1_m3 JP2_m3 zeros(3,2)];

JO3_m3=kr3*z_2;
JM3.JO_m3=[JO1_l3 JO2_l3 JO3_m3 zeros(3,1)];


% Motore 4
pm4=T_03(1:3,1:3)*COM_m4;
I_m4=T_03(1:3,1:3)*I_m4_b*T_03(1:3,1:3)';

Inerzia_motori.I_m4 = I_m4;%&%&%&%&%&%

JM4= struct;

JP1_m4=cross(z_0,pm4-p_0);
JP2_m4=cross(z_1,pm4-p_1);
JP3_m4=cross(z_2,pm4-p_2);
JM4.JP_m4=[JP1_m4 JP2_m4 JP3_m4 zeros(3,1)];

JO4_m4=kr4*z_3;
JM4.JO_m4=[JO1_l4 JO2_l4 JO3_l4 JO4_m4];

JM = struct;
JM.JM1 = JM1;
JM.JM2 = JM2;
JM.JM3 = JM3;
JM.JM4 = JM4;
end