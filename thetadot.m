function thetadot = thetadot(w, theta)
% Time derivative of Rotation Vector

a = w + cross(0.5 * theta, w);

b = (1/norm(theta)^2) * (1 - (norm(theta) / 2) * cot(norm(theta) / 2)) * theta;

c = cross(theta, w);

thetadot = a + cross(b, c);

end