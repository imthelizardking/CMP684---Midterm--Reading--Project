%
%% Init.s
clear all; clc; close all;
nSimDuration = 10; del = 1;
nK = 10; % 10 time steps
nPredictionStep = 2; % horizon length = 2
nIterMax = 10; % num of max iterations
nInputs = 6; nOutputs = 6; % 6 joints, 6 dof
q = zeros(nK, nInputs, nIterMax); % q is (Sim_Len, nJnts, nIter) (10, 6, 10)
J = zeros(nInputs,nOutputs,nSimDuration,nIterMax);
Y = zeros(nSimDuration, nOutputs, nIterMax);
Phi = zeros(nPredictionStep*size(J));
delta_Q = zeros(nPredictionStep*nInputs, 1);
I = [eye(nInputs), eye(nInputs);...
     eye(nInputs), eye(nInputs)];
E = [];
for i=1:nPredictionStep
    E = cat(1, E, eye(nOutputs));
end
Y_star = zeros(nSimDuration,nOutputs);
q(1,:,1,1) = [-pi/4,0,0,0,-pi/2,0]; q_final = [pi/4,0,0,0,-pi/2,0]';
q(2,:,1,1) = [-pi/4,0,0,0,-pi/2,0];
for t=1:10 % 10 time step
    bExit = false;
    while bExit==false
        J(:,:,1,i,t) = calcJacobian(q(1,:,i-1,t));
        J(:,:,2,i,t) = calcJacobian(q(2,:,i-1,t));
        Phi(:,:,i,t) = [ inv(J(:,:,1,i,t)),   zeros(size(inv(J(:,:,1,i,t))));...
                        -inv(J(:,:,1,i,t)), inv(J(:,:,1,i,t))];
        temp_Y_tm = calcTransform(q(1,:,i-1,t));     
        y_temp = calcPose(temp_Y_tm);
        
    end
    
    
    
    
end