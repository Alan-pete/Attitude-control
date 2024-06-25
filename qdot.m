function qdot = qdot(w, q)
% Time derivative of quaternion

w1.s = 0;
w1.v = w;

qdot1 = qX(w1,q);

qdot.s = 1/2 * qdot1.s;
qdot.v = 1/2 * qdot1.v;

end