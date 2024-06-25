function wbidot_B = wdot(T, J, w)
% Angular velocity derivative using Euler's equation for rotational motion

wbidot_B = J\(T - cross(w,J*w));

end