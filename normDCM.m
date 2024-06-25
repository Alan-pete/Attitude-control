function A = normDCM(A_new)
% normalizing the DCM

E = 1/2 * ((A_new * A_new') - eye(3,3));

A = (eye(3,3) - E) * A_new;

end