%% Calcolo dello jacobiano geometrico del manipolatore planare a 4 G.d.L

function Ja = calcolo_jacobiano_simbolico
%Definisco, in forma simbolica, le tre variabili di giunto e le lunghezze dei
%link
syms q1 q2 q3 q4 'real'
syms a1 a2 a3 a4 'real'

%Calcolo le matrici di trasformazione omogenea fra i vari link
%Chiamo la function specifica che mi assembla direttamente la matrice di
%trasformazione della terna i-esima rispetto a quella i-1-esima
T0_1=calc_DH(0,a1,0,q1);
T1_2=calc_DH(0,a2,0,q2);
T2_3=calc_DH(0,a3,0,q3);
T3_4=calc_DH(0,a4,0,q4);

%Calcolo la matrice di trasformazione omogenea complessiva
T0_2=T0_1*T1_2;
T0_3=T0_2*T2_3;
T0_4=T0_3*T3_4;
%T0_4 è la funzione di cinematica diretta del manipolatore

%Calcolo dello Jacobiano geometrico
%Ho solo giunti rotoidali
%estrapolo i parametri necessari
z0=[0 0 1]';
z1=T0_1(1:3,3);
z2=T0_2(1:3,3);
z3=T0_3(1:3,3);
z4=T0_4(1:3,3);

p0=[0 0 0]';
p1=T0_1(1:3,4);
p2=T0_2(1:3,4);
p3=T0_3(1:3,4);
p4=T0_4(1:3,4);

%Calcolo le differenze tra le origini
p0_4=p4-p0;
p1_4=p4-p1;
p2_4=p4-p2;
p3_4=p4-p3;

%Calcolo i prodotti vettoriali necessari fra gli elementi e poi costituisco
%le colonne del jacobiano geometrico.
J1 = [cross(z0,p0_4); z0];
J2 = [cross(z1,p1_4); z1];
J3 = [cross(z2,p2_4); z2];
J4 = [cross(z3,p3_4); z3];
%Compongo ora il jacobiano geometrico
J=[J1 J2 J3 J4];
%simplify semplifica le equazioni: ho seni e coseni dalla cinematica
%diretta.
J=simplify(J);

%Calcolo gli angoli di Eulero (XYZ) perchè mi serve la ...
%funzione di cinematica diretta. Non ho scelto la ZYZ perchè essendo
%planare non ho rotazioni intorno all'asse y, quindi avrei due rotazioni
%consecutive intorno a z (NO).
psi=atan2(T0_4(3,2),T0_4(3,3));
theta=atan2(-T0_4(3,1),sqrt(T0_4(3,2)^2+T0_4(3,3)^2));
phi=atan2(T0_4(2,1),T0_4(1,1));

%Vettore uscita parametri di Eulero
E=[psi;theta;phi];
%simplify(E);

%x è la posa dell'organo terminale
x=[p4;E];

%Calcolo jacobiano analitico

%La prima colonna si ottiene differenziando il vettore x rispetto al primo
%angolo di giunto e così via.
Ja_1=simplify(diff(x,'q1'));
Ja_2=simplify(diff(x,'q2'));
Ja_3=simplify(diff(x,'q3'));
Ja_4=simplify(diff(x,'q4'));

%Compongo quindi lo Jacobiano analitico
Ja=[Ja_1, Ja_2, Ja_3, Ja_4];
end
