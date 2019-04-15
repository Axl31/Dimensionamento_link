function [XY1, XY2, XY3, XY4]=kin_man_rid_progetto(q,a)

q12 = q(1)+q(2);
q123 = q12 + q(3);
q1234 = q123 + q(4);


XY1=[a(1)*cos(q(1));
    a(1)*sin(q(1));
    q(1)];

XY2=[a(1)*cos(q(1))+a(2)*cos(q12);
    a(1)*sin(q(1))+a(2)*sin(q12);
    q12];

XY3=[a(1)*cos(q(1))+a(2)*cos(q12)+a(3)*cos(q123);
    a(1)*sin(q(1))+a(2)*sin(q12)+a(3)*sin(q123);
    q123];

XY4= [a(1)*cos(q(1))+a(2)*cos(q12)+a(3)*cos(q123)+a(4)*cos(q1234);
      a(1)*sin(q(1))+a(2)*sin(q12)+a(3)*sin(q123)+a(4)*sin(q1234);
      q1234];


end