function [ qout ] = qX( q, p )
%QX Calculate the product of two quaternions. (Cross Notation)
%   Hint: Use the scalar and vector portions directly

qout.s = (q.s * p.s) - (transpose(q.v) * p.v);
qout.v = (q.s * p.v) + (p.s * q.v) - cross(q.v, p.v);

end