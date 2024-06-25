function [ e, theta ] = A2e( A )
%A2E Convert a DCM to Eigen Axis/Angle Representation
%   Hint: Use eq 2.113, 2.114, and 2.115.  Handle theta = 0 and theta = pi.
% When theta = pi normalize the column with the largest magnitude
% When theta = 0 choose any unit vector for e (e is undefined)

% Eq 2.113
theta = acos((trace(A) - 1) / 2);

if theta == 0
    
    e = [1;0;0];
    
elseif theta == pi
        
        % Eq 2.115
        eeT = (A + eye(3))/2;
        
        [val, i] = max([norm(eeT(:,1)),norm(eeT(:,2)),norm(eeT(:,3))]);
        
        if val == 0
            
            e = [0;0;0];
            
        else
            
            e = eeT(:,i)/val;
            
        end
        
else
    
    % Eq 2.114
    e = 1/(2*sin(theta))*[A(2,3)-A(3,2);A(3,1)-A(1,3);A(1,2)-A(2,1)];
        
end

end

