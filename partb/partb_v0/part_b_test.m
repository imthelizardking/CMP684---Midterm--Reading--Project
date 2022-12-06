%% Init.s
q = zeros(nK, nInputs, nIterMax);
q(0,:,:) = [-pi/4,0,0,0,-pi/2,0]';



nIterMax = 10; nInputs = 6; nOutputs = 6;
nPredictionStep = 2;
I = [eye(nInputs), eye(nInputs);...
     eye(nInputs), eye(nInputs)];
E = [];
for i=1:nPredictionStep
    E = cat(1, E, eye(nOutputs));
end
nK = 10; nPredictionStep = 2;
%x_star = ; y_star =; z_star = ;
Y_star = zeros(nK+1, nOutputs);
%Y_star = [x_star; y_star; z_star];
%% Algorithm:
for k = 1:nK % 1 or 0?
    %Y_n() = ;
    i=1;
    bExit = false;
    while (i<=nIterMax && bExit==false)
        %% Calc.
        Y_star_horizon = [];
        for j=1:nPredictionStep
            Y_star_horizon = [Y_star_horizon; Y_star(k+j,:)];
        end
        J(:,:,k,i) = calcJacobian(q(k,:,i-1));
        J(:,:,k+1,i) = calcJacobian(q(k+1,:,i-1));
        Phi(:,:,k,i) = [inv(J(:,:,k,i)), 0;...
                        -inv(J(:,:,k+1,i)), inv(J(:,:,k+1,i))];
        temp = calcTransform(q(k,:,i-1));
        y(k,:,i) = temp(1:3,4);
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