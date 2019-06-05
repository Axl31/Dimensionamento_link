function tau= PID_controller(Q, Qdot, Qi, Qdes, Qdotdes, Qides,K_,D_,I_)
%tau= PID_controller(Q, Qdot, Qi, Qdes, Qdotdes, Qides,K_,D_,I_)


K=diag(K_);

D=diag(D_);

I=diag(I_);

% La nostra legge di controllo è la seguente: 
tau=K*(Qdes-Q)+D*(Qdotdes-Qdot)+I*(Qides-Qi);
% dove (Qides-Qi) sono gli integrali della nostra Q desiderata e quella
% attuale ossia l'integrale dell'errore di posizione 

end