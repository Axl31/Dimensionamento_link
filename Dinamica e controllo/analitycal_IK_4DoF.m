function Q=analitycal_IK_4DoF(p,theta,a1,a2,a3)

pw_x=p(1)-a3*cos(theta);
pw_y=p(2)-a3*sin(theta);

c2=(pw_x^2+pw_y^2-a1^2-a2^2)/(2*a1*a2);
s2=sqrt(1-c2^2);

s1=((a1+a2*c2)*pw_y-a2*s2*pw_x)/(pw_x^2+pw_y^2);
c1=((a1+a2*c2)*pw_x+a2*s2*pw_y)/(pw_x^2+pw_y^2);

Q=[];
if (isreal(s1) && isreal(s2) && isreal(c1) && isreal(c2))
    q1=atan2(s1,c1);
    q2=atan2(s2,c2);
    q3=theta-q1-q2;
    Q_=[q1 q2 q3];
    X = direct_kinematics_3DoF(Q_,a1,a2,a3);
    if(abs(X-[p(1);p(2);theta])<1e-3)
        
        Q=[q1 q2 q3];
        
    end
    
end

end