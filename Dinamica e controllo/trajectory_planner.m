%This function calculates the actual desired robot position, using a fifth
%order polynomial function. 
%Input parameters are initial position (Pi), final position (Pf), initial 
%time (ti), final time (tf) and current time (t).
%Output variables are robot desired position (XYd) and velocity (XYddot).

function [XYd ,XYddot] =trajectory_planner(u)
pi=u(1:2);
pf=u(3:4);
ti=u(5);
tf=u(6);
t=u(7);
A=[ti^5 ti^4 ti^3 ti^2 ti 1;
   tf^5 tf^4 tf^3 tf^2 tf 1;
   5*ti^4 4*ti^3 3*ti^2 2*ti 1 0;
   5*tf^4 4*tf^3 3*tf^2 2*tf 1 0;
   20*ti^3 12*ti^2 6*ti 2 0 0;
   20*tf^3 12*tf^2 6*tf 2 0 0];
B=[0;norm(pi-pf);0;0;0;0];
a=inv(A)*B;
s=a(1)*t^5+a(2)*t^4+a(3)*t^3+a(4)*t^2+a(5)*t+a(6);
sdot=5*a(1)*t^4+4*a(2)*t^3+3*a(3)*t^2+2*a(4)*t+a(5);
if t<ti
    XYd=pi;
    XYddot=[0;0];
else
    if t<=tf
    XYd=pi+(s/norm(pi-pf))*(pf-pi);
    XYddot=(sdot/norm(pi-pf))*(pf-pi);
    else
    XYd=pf;
    XYddot=[0;0];
    end
end
%OUT=[XYd;XYddot];
%OUT=XYd;
end