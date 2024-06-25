function [ A ] = euler2A( e1, e2, e3, phi, theta, psi )
%EULER2A Convert a User Specified Euler Angle Sequence to a DCM
%   Hint: e1, e2, e3 specify the Euler Angle Sequence (3,2,1 is ZYX)
% phi is the angle associated with e1
% theta is the angle associated with e2
% psi is the angle associated with e3
% Create three direction cosine matrices: A1(e1,phi), A2(e2,theta), A3(e3,psi)
% A = A3*A2*A1

% A1 Rotation
if e1 == 1
    A1 = [1, 0, 0;
        0, cos(phi), sin(phi);
        0, -sin(phi), cos(phi)];
elseif e1 == 2
    A1 = [cos(phi), 0, -sin(phi);
        0, 1, 0;
        sin(phi), 0, cos(phi)];
elseif e1 == 3
    A1 = [cos(phi), sin(phi), 0;
        -sin(phi), cos(phi), 0;
        0, 0, 1];
end

% A2 Rotation
if e2 == 1
    A2 = [1, 0, 0;
        0, cos(theta), sin(theta);
        0, -sin(theta), cos(theta)];
elseif e2 == 2
    A2 = [cos(theta), 0, -sin(theta);
        0, 1, 0;
        sin(theta), 0, cos(theta)];
elseif e2 == 3
    A2 = [cos(theta), sin(theta), 0;
        -sin(theta), cos(theta), 0;
        0, 0, 1];
end

% A3 Rotation
if e3 == 1
    A3 = [1, 0, 0;
        0, cos(psi), sin(psi);
        0, -sin(psi), cos(psi)];
elseif e3 == 2
    A3 = [cos(psi), 0, -sin(psi);
        0, 1, 0;
        sin(psi), 0, cos(psi)];
elseif e3 == 3
    A3 = [cos(psi), sin(psi), 0;
        -sin(psi), cos(psi), 0;
        0, 0, 1];
end

A = A3*A2*A1;

end
