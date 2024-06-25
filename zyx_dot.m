function zyxdot = zyx_dot(w, zyx)
% Time derivative of Euler Angles
% zyx = [phi; theta; psi]

B_321 = [0, sin(zyx(3)), cos(zyx(3));
        0, cos(zyx(2)) * cos(zyx(3)), -cos(zyx(2)) * sin(zyx(3));
        cos(zyx(2)), sin(zyx(2)) * sin(zyx(3)), sin(zyx(2)) * cos(zyx(3))];

B = 1/cos(zyx(2)) * B_321;

zyxdot = B * w;

end