
function J=J_man_rid(Q,r1,r2,r3)
q1=Q(1);
q2=Q(2);
q3=Q(3);


J=[-r1*sin(q1)-r2*sin(q1+q2)-r3*sin(q1+q2+q3) -r2*sin(q1+q2)-r3*sin(q1+q2+q3) -r3*sin(q1+q2+q3);
    r1*cos(q1)+r2*cos(q1+q2)+r3*cos(q1+q2+q3)  r2*cos(q1+q2)+r3*cos(q1+q2+q3) r3*cos(q1+q2+q3);
   1 1 1];
end