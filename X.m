function [ vX ] = X( v )
%  X Calculate a Cross Product Matrix from a Vector
%   Hint: Use eq 2.55

vX = [0, -v(3), v(2);
    v(3), 0, -v(1);
    -v(2), v(1), 0];

end

