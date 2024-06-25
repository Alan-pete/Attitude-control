function [e, angle] = theta2AA(theta)
% Rotation vector to Eigen Axis/Angle

angle = norm(theta);
e = theta / angle;

if angle == 0
    e = [1; 0; 0];

elseif angle > pi && angle < 2*pi
    angle = 2*pi - angle;
    e = -e;

elseif angle >= 2*pi
    angle = mod(angle, 2*pi);
    
end

end