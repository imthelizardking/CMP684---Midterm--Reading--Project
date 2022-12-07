%% Init.s
nIterationMax = 10; % num of max iterations
nSimDuration = 10; % simulation duration
nPredictionStep = 2; % horizon length = 2
nInputs = 6; nOutputs = 6; % 6 joints, 6 dof
I = [eye(nInputs), eye(nInputs);...
     eye(nInputs), eye(nInputs)];
Y = zeros(nPredictionStep,nInputs,nIterationMax,nSimDuration); % Task space vector, size(2,6,10,10)
q = zeros(nPredictionStep,nInputs,nIterationMax,nSimDuration); % Joint space vector, size(2,6,10,10)
J = zeros(nOutputs, nInputs,nPredictionStep,nIterationMax,nSimDuration); % Jacobian
Phi = zeros(nPredictionStep*size(J));
%% Init time zero
q0 = [-pi/4,0,0,0,-pi/2,0]'; % begining joint angles
qF = [ pi/4,0,0,0,-pi/2,0]'; % goal joint angles
Y0 = calcPose(q0); % beginning task space variables
YF = calcPose(qF); % goal task space variables
% Init. iteration i=1 of step k=1
Y(1,:,1,1) = Y0; % task space variables @ k=1, i=1
Y(2,:,1,1) = Y0; % task space variables @ k=1, i=1
q(1,:,1,1) = q0; % joint angles @ k=1, i=1
q(2,:,1,1) = q0; % joint angles @ k=1, i=1
%% Main Loop
for k=1:10 % simulation loop
    bExit = false;
    for i=2:nIterationMax
        Y(1,:,i,k) = calcPose(q(1,:,i-1,k));
        Y(2,:,i,k) = calcPose(q(2,:,i-1,k));
        J(:,:,1,i,k) = calcJacobian(q(1,:,i-1,k));
        J(:,:,2,i,k) = calcJacobian(q(1,:,i-1,k));
        Phi(:,:,i,k) = [inv(J(:,:,1,i,k)), zeros(size(inv(J(:,:,1,i,k))));...
                        -inv(J(:,:,2,i,k)), inv(J(:,:,2,i,k))];
        % Calculate Predictive Error:
        err_pred = calculatePredictiveError(Y(1,:,i,k), Y(2,:,i,k), q(1,:,i-1,k), q(2,:,i-1,k), Y0(4:6), YF(4:6), Y0, YF, k);
        delta_Q = inv(Phi(:,:,i,k))*err_pred;
        temp1 = [q(1,:,i-1,k)';q(1,:,i-1,k)'] + I * delta_Q;
        q(1,:,i,k) = temp1(1:6); q(2,:,i,k) = temp1(7:12);
    end
    q(1,:,1,k+1) = q(1,:,10,k); q(2,:,1,k+1) = q(2,:,10,k);
    Y(1,:,1,k+1) = Y(1,:,10,k); Y(2,:,1,k+1) = Y(2,:,10,k);
end

%% Plots