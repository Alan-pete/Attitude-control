function [ qout ] = qUnit( q )
%QNORMALIZE Normalizes a quaternion
%   Hint: Include the vector and scalar portions

mag = sqrt(q.s^2 + q.v(1)^2 + q.v(2)^2 + q.v(3)^2);

qout.s = q.s / mag;
qout.v = q.v / mag;

end

