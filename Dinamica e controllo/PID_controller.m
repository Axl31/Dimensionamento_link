function tau= PID_controller(Q, Qdot, Qi, Qdes, Qdotdes, Qides,K_,D_,I_)

K=diag(K_);

D=diag(D_);

I=diag(I_);


tau=K*(Qdes-Q)+D*(Qdotdes-Qdot)+I*(Qides-Qi);
% dove (Qides-Qi) è l'itegrale della nostra Q desiderata e quella attuale

end