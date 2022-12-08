%% Init.s
clear all; clc; close all;
nK = 10; nPredictionStep = 2;
nIterMax = 10; nInputs = 6; nOutputs = 6;
q = zeros(nK, nInputs, nIterMax);
q(1,:,1) = [-pi/4,0,0,0,-pi/2,0];




I = [eye(nInputs), eye(nInputs);...
     eye(nInputs), eye(nInputs)];
E = [];
for i=1:nPredictionStep
    E = cat(1, E, eye(nOutputs));
end

%x_star = ; y_star =; z_star = ;
Y_star = zeros(nK+1, nOutputs);
%Y_star = [x_star; y_star; z_star];
%% Algorithm:
for k = 2:nK % 2 or 1 or 0?
    %Y_n() = ;
    i=2;
    bExit = false;
    while (i<=nIterMax && bExit==false)
        %% Calc matrices (Eqn.(45) > (54)
        T_star(:,:,k,i) = [cos(b_star(k))*cos(g_star(k)), cos(g_star(k))*sin(a_star(k))*sin(b_star(k))-cos(a_star(k))*sin(g_star(k)), sin(a_star(k))*sin(g_star(k))+cos(a_star(k))*cos(g_star(k))*sin(b_star(k));...
                           cos(b_star(k))*sin(g_star(k)), cos(a_star(k))*cos(g_star(k))+sin(a_star(k))*sin(g_star(k))*sin(b_star(k)), cos(a_star(k))*sin(b_star(k))*sin(g_star(k))-cos(g_star(k))*sin(a_star(k));...
                           -sin(b_star(k)), cos(b_star(k))*sin(a_star(k)), cos(a_star(k))*cos(b_star(k))];
        A_star(:,:,k,i) = T_star(1:3,1:3,k,i);
        D(:,:,k,i) = A_star(:,:,k+1,i) * inv(A_star(:,:,k,i));
        K(:,k,i) = [D(3,2,k,i) - D(2,3,k,i);...
            D(1,3,k,i) - D(3,1,k,i);...
            D(2,1,k,i) - D(1,2,k,i)] / (2*sin(Theta(k,i)));
        AngleDiff(:,k,i) = K(:,k,i) * Theta(k,i);
        %% Calc.
        Y_star_horizon = [];
        for j=1:nPredictionStep
            Y_star_horizon = [Y_star_horizon'; Y_star(k+j,:)'];
        end
        J(:,:,k,i) = calcJacobian(q(k,:,i-1));
        J(:,:,k+1,i) = calcJacobian(q(k+1,:,i-1));
        Phi(:,:,k,i) = [inv(J(:,:,k,i)), zeros(size(inv(J(:,:,k,i))));...
                        -inv(J(:,:,k+1,i)), inv(J(:,:,k+1,i))];
        temp = calcTransform(q(k,:,i-1));
        y(k,:,i) = temp(1:3,4); % also angles!

        


        delta_Q(:,:,k,i) = inv(Phi(:,:,k,i)) * (Y_star_horizon - E*y(k,:,i));
        temp1 = [q(k,:,i-1); q(k+1,:,i-1)] + I * delta_Q(:,:,k,i);  %[q(k,:,i); q(k+1,:,i)]
        q(k,:,i)    = temp1(1:nInputs);
        q(k+1,:,i)  = temp1(nInputs+1:end); 
        %% Stop or Cont.
        i = i + 1;
%         if ()
%             bExit = true;
%         end
    end
end