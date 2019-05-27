
function [q , qdot] =joint_planner(qi,qf,ti,tf,t)

A=[ti^5 ti^4 ti^3 ti^2 ti 1;
   tf^5 tf^4 tf^3 tf^2 tf 1;
   5*ti^4 4*ti^3 3*ti^2 2*ti 1 0;
   5*tf^4 4*tf^3 3*tf^2 2*tf 1 0;
   20*ti^3 12*ti^2 6*ti 2 0 0;
   20*tf^3 12*tf^2 6*tf 2 0 0];
B=[qi;qf;0;0;0;0];%[0;abs(qf-qi);0;0;0;0] ma non è la formulazione corretta perchè non stiamo 
% descrivendo la traiettoria tramite s (spazio operativo) ma direttamente
% nello spazio dei giunti
a=inv(A)*B;
q=a(1)*t^5+a(2)*t^4+a(3)*t^3+a(4)*t^2+a(5)*t+a(6);
qdot=5*a(1)*t^4+4*a(2)*t^3+3*a(3)*t^2+2*a(4)*t+a(5);

end