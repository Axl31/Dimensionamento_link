%% Calcolo della matrice di inerzia B
%% Link 1 e motore 1
B1_l=simplify(M_1*(JP_l1)'*JP_l1 + (JO_l1)'*I_01*JO_l1);
B1_m=(M_m1*(JP_m1)'*JP_m1 + (JO_m1)'*I_m1*JO_m1);
B1=B1_l+B1_m;

%% Link e motore 2
B2_l=simplify(M_2*(JP_l2)'*JP_l2+(JO_l2)'*I_02*JO_l2);
B2_m=simplify(M_m2*(JP_m2)'*JP_m2+(JO_m2)'*I_m2*JO_m2);
B2=B2_l+B2_m;

%% Link e motore 3
B3_l=simplify(M_3*(JP_l3)'*JP_l3+(JO_l3)'*I_03*JO_l3);
B3_m=simplify(M_m3*(JP_m3)'*JP_m3+(JO_m3)'*I_m3*JO_m3);
B3=B3_l+B3_m;


%% Link e motore 4
B4_l=simplify(M_4*(JP_l4)'*JP_l4+(JO_l4)'*I_04*JO_l4);
B4_m=simplify(M_m4*(JP_m4)'*JP_m4 + (JO_m4)'*I_m4*JO_m4);
B4=B4_l+B4_m;


%% Matrice di inerzia B complessiva
B=simplify(B1+B2+B3+B4);

%% Matrice C
syms C 'real'
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
syms G 'real'
G=vpa(zeros(4,1));
g_m=vpa(zeros(4,1));
g_l=vpa(zeros(4,1));

for i=1:4
   g_m(i)=simplify(M_m1*g0'*JP_m1(:,i)+M_m2*g0'*JP_m2(:,i)+ ...
          M_m3*g0'*JP_m3(:,i)+M_m4*g0'*JP_m4(:,i));
    
   g_l(i)=simplify(M_1*g0'*JP_l1(:,i)+M_2*g0'*JP_l2(:,i)+ ...
          M_3*g0'*JP_l3(:,i)+M_4*g0'*JP_l4(:,i));
   G(i)=simplify(g_l(i)+g_m(i));
end