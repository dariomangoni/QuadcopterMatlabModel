function [vettore_earthframe] = body_to_earth(vettore_bodyframe, quaternione)

rotation_matrix=2*[0.5-quaternione(3)^2-quaternione(4)^2,         quaternione(2)*quaternione(3)-quaternione(1)*quaternione(4), quaternione(2)*quaternione(4)+quaternione(1)*quaternione(3);...
                           quaternione(2)*quaternione(3)+quaternione(1)*quaternione(4), 0.5-quaternione(2)^2-quaternione(4)^2,         quaternione(3)*quaternione(4)-quaternione(1)*quaternione(2);...
                           quaternione(2)*quaternione(4)-quaternione(1)*quaternione(3), quaternione(3)*quaternione(4)+quaternione(1)*quaternione(2), 0.5-quaternione(2)^2-quaternione(3)^2       ];

vettore_earthframe = rotation_matrix * vettore_bodyframe;
end