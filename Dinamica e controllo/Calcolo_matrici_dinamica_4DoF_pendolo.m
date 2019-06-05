function [B,C,G]= Calcolo_matrici_dinamica_4DoF_pendolo(JL,JM,Inerzia_link,Inerzia_motori,masse_link,masse_motori,Q,Q_dot,g0)

JL1=JL.JL1;
JL2=JL.JL2;
JL3=JL.JL3;
JL4=JL.JL4;

JM1=JM.JM1;
JM2=JM.JM2;
JM3=JM.JM3;
JM4=JM.JM4;

M_1=masse_link(1);
M_2=masse_link(2);
M_3=masse_link(3);
M_4=masse_link(4);

M_m1= masse_motori(1);
M_m2= masse_motori(2);
M_m3= masse_motori(3);
M_m4= masse_motori(4);

I_01= Inerzia_link.I_01;
I_02= Inerzia_link.I_02;
I_03= Inerzia_link.I_03;
I_04= Inerzia_link.I_04;

I_m1= Inerzia_motori.I_m1;
I_m2= Inerzia_motori.I_m2;
I_m3= Inerzia_motori.I_m3;
I_m4= Inerzia_motori.I_m4;

%% Calcolo della matrice di inerzia B
%% Link 1 e motore 1
B1_l=simplify(M_1*(JL1.JP_l1)'*JL1.JP_l1 + (JL1.JO_l1)'*I_01*JL1.JO_l1);
B1_m=(M_m1*(JM1.JP_m1)'*JM1.JP_m1 + (JM1.JO_m1)'*I_m1*JM1.JO_m1);
B1=B1_l+B1_m;

%% Link e motore 2
B2_l=simplify(M_2*(JL2.JP_l2)'*JL2.JP_l2+(JL2.JO_l2)'*I_02*JL2.JO_l2);
B2_m=simplify(M_m2*(JM2.JP_m2)'*JM2.JP_m2+(JM2.JO_m2)'*I_m2*JM2.JO_m2);
B2=B2_l+B2_m;

%% Link e motore 3
B3_l=simplify(M_3*(JL3.JP_l3)'*JL3.JP_l3+(JL3.JO_l3)'*I_03*JL3.JO_l3);
B3_m=simplify(M_m3*(JM3.JP_m3)'*JM3.JP_m3+(JM3.JO_m3)'*I_m3*JM3.JO_m3);
B3=B3_l+B3_m;


%% Link e motore 4
B4_l=simplify(M_4*(JL4.JP_l4)'*JL4.JP_l4+(JL4.JO_l4)'*I_04*JL4.JO_l4);
B4_m=simplify(M_m4*(JM4.JP_m4)'*JM4.JP_m4 + (JM4.JO_m4)'*I_m4*JM4.JO_m4);
B4=B4_l+B4_m;


%% Matrice di inerzia B complessiva
B=simplify(B1+B2+B3+B4)%;

%% Matrice C
sym C 'real';
C=vpa(zeros(4));

for i=1:4
    for j=1:4
        for k=1:4
           a=simplify(diff(B(i,j),Q(k)));
           b=simplify(diff(B(i,k),Q(j)));
           c=simplify(diff(B(j,k),Q(i)));
           C(i,j)=simplify(C(i,j)+0.5*(a+b-c)*Q_dot(k));
        end
        
    end
end

%% Calcolo vettore G
sym G 'real';
G=vpa(zeros(4,1));
g_m=vpa(zeros(4,1));
g_l=vpa(zeros(4,1));

for i=1:4
   g_m(i)=simplify(M_m1*g0'*JM1.JP_m1(:,i)+M_m2*g0'*JM2.JP_m2(:,i)+ ...
          M_m3*g0'*JM3.JP_m3(:,i)+M_m4*g0'*JM4.JP_m4(:,i));
    
   g_l(i)=simplify(M_1*g0'*JL1.JP_l1(:,i)+M_2*g0'*JL2.JP_l2(:,i)+ ...
          M_3*g0'*JL3.JP_l3(:,i)+M_4*g0'*JL4.JP_l4(:,i));
   G(i)=simplify(g_l(i)+g_m(i));
end