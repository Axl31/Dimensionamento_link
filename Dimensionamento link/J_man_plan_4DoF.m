%% Algoritmo di inversione cinematica con l'inversa dello jacobiano per manipolatore planare a 4DoF
% Dobbiamo imporre quindi necessariamente delle condizioni iniziali per la
% posizione dei 4 giunti. 
function J= J_man_plan_4DoF(q,a)

q12 = q(1)+q(2);
q123 = q12 + q(3);
q1234 = q123 + q(4);

% calcolo del jacobiano
J=[-a(1)*sin(q(1))-a(2)*sin(q12)-a(3)*sin(q123)-a(4)*sin(q1234) -a(2)*sin(q12)-a(3)*sin(q123)-a(4)*sin(q1234) ...
    -a(3)*sin(q123)-a(4)*sin(q1234) -a(4)*sin(q1234);
    a(1)*cos(q(1))+a(2)*cos(q12)+a(3)*cos(q123)+a(4)*cos(q1234)  a(2)*cos(q12)+a(3)*cos(q123)+a(4)*cos(q1234) ...
    a(3)*cos(q123)+a(4)*cos(q1234) a(4)*cos(q1234); 1 1 1 1];

end