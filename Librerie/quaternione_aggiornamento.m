function quat_out = quaternione_aggiornamento(quat_in, omega, dt)
blocco = 0.5*norm(omega)*dt;
if blocco~=0
    omegaquat = [ cos(blocco) ; (0.5*omega*dt)/blocco*sin(blocco) ];
    quat_out=quaternione_moltiplicazione(omegaquat,quat_in);
else
    quat_out = quat_in;
end

end