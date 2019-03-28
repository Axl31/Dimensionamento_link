function Q=cinematica_inversa_4gdl(p, theta, a, q4)
%& per passara dallo spazio operativo a quello dei giunti

%Applicazione del problema cinematico inverso (Noti p e theta->si vuole ricavare Q)
%& guarda esempio del manipolatore a 3gdl fatto in classe
%q4= deg2rad(7);

    pw_x = p(1) - a(3) * cos(theta - q4) - a(4) * cos(theta);
    pw_y = p(2) - a(3) * sin(theta - q4) - a(4) * sin(theta);

    c2 = ( pw_x^2 + pw_y^2 - a(1)^2 - a(2)^2) / (2 * a(1) * a(2));
    s2 = sqrt( 1 - c2^2);

    s1=((a(1) + a(2)*c2) * pw_y - a(2) * s2 * pw_x) / (pw_x^2 + pw_y^2);
    c1=((a(1) + a(2)*c2) * pw_x + a(2) * s2 * pw_y) / (pw_x^2 + pw_y^2);

    Q = [];
    %& se non sono reali quella soluzione è da scartare
    if (isreal(s1) && isreal(s2) && isreal(c1) && isreal(c2))
        q1 = atan2(s1,c1);
        q2 = atan2(s2,c2);
        q3 = theta - q1 - q2 - q4;
        %ho una configurazione di angoli noti fi,theta e voglio
        %ricavare theta 1,2,3 ossia gli angoli di giunto. Se conosco tutto devo
        %riapplicare la cinematica diretta per vedere se la posizione che ho
        %trovato è quella desiderata poichè questi algoritmi possono creare
        %degli errori; quindi impongo che la differenza tra la posizione calcolata e quella desiderata sia inferiore a 10^(-3).
        % se cosi non fosse scarto quella combinazione
        X = cinematica_diretta_4gdl([q1, q2, q3, q4], a);%viene applicata la cinematica diretta

        if abs(X-[p(1);p(2);theta]) < 1e-3
            Q = [q1 q2 q3 q4];  
        end

    end
end