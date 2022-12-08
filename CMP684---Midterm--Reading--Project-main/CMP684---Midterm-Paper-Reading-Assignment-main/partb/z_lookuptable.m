%
% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% 1) Calculate Delta_Q
%       Need: Phi > Jacobian
%       Need: (Y_n(k+1), y(k))
%               > Y_n(k+1) > Angles from transformations
%                          > Positions from J*Delta_Q
% 2) Using Delta_Q and Jacobian, calculate Y(k+1)
%
%
%
%
T_star(:,:,k,i) = [cos(b_star(k))*cos(g_star(k)), cos(g_star(k))*sin(a_star(k))*sin(b_star(k))-cos(a_star(k))*sin(g_star(k)), sin(a_star(k))*sin(g_star(k))+cos(a_star(k))*cos(g_star(k))*sin(b_star(k));...
                   cos(b_star(k))*sin(g_star(k)), cos(a_star(k))*cos(g_star(k))+sin(a_star(k))*sin(g_star(k))*sin(b_star(k)), cos(a_star(k))*sin(b_star(k))*sin(g_star(k))-cos(g_star(k))*sin(a_star(k));...
                   -sin(b_star(k)), cos(b_star(k))*sin(a_star(k)), cos(a_star(k))*cos(b_star(k))];
A_star(:,:,k,i) = T_star(1:3,1:3,k,i);
D(:,:,k,i) = A_star(:,:,k+1,i) * inv(A_star(:,:,k,i));
Theta(k,i) = acos((D(1,1,k,i)+D(2,2,k,i)+D(3,3,k,i)-1)/2);
K(:,k,i) = [D(3,2,k,i) - D(2,3,k,i);...
            D(1,3,k,i) - D(3,1,k,i);...
            D(2,1,k,i) - D(1,2,k,i)] / (2*sin(Theta(k,i)));
AngleDiff(:,k,i) = K(:,k,i) * Theta(k,i);

%% Control:

% Calc. J_inv:
J_inv(:,:,k,1) = calcJacobian_inv(alpha(k-1, N), beta(k-1, N), gamma(k-1, N));
J_inv(:,:,k,2) = calcJacobian_inv(alpha(k, N), beta(k, N), gamma(k, N));
% Calc. Phi_inv:
Phi_inv(:,:,k,i) = [J_inv(:,:,k,i), 0; -J_inv(:,:,k+1,i), J_inv(:,:,k+1,i)];

% Calc. Delta_Q:
Delta_Q(k,:,i) = Phi_inv(:,:,k,i) * [Y_star(k+1, :)' - E*y(k)'];


