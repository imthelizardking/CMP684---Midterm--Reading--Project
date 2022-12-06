%% Init.s
clear all; clc; close all;
nSimDuration = 10; del = 999999;
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
q(1,:,1) = [-pi/4,0,0,0,-pi/2,0]; q_final = [pi/4,0,0,0,-pi/2,0]';
temp_pose_init = calcTransform(q(1,:,1)); temp_pose_final = calcTransform(q_final);
pose_init = calcPose(temp_pose_init); pose_final = calcPose(temp_pose_final);
%% Loops:
for k=1:nSimDuration-1
    i = 2;
    bExit = false;
    if k==1
        tempTrnsMtrx = calcTransform(q(k,:,1));
        Y_star(k,:) = calcPose(tempTrnsMtrx);
    else
        Y_star(k,:) = calcPath(pose_init,pose_final,q(1,:,1),q_final,k);
    end
    while (i<=nIterMax && bExit==false)
        J(:,:,k,i) = calcJacobian(q(k,:,i-1));
        J(:,:,k+1,i) = calcJacobian(q(k+1,:,i-1)); % ?
        Phi(:,:,k,i) = [inv(J(:,:,k,i)), zeros(size(inv(J(:,:,k,i))));...
                        -inv(J(:,:,k+1,i)), inv(J(:,:,k+1,i))];
        temp_Y = calcTransform(q(k,:,i-1));      
        Y(k,:,i) = calcPose(temp_Y);
        delta_Q(:,:,k,i) = inv(Phi(:,:,k,i)) * ([Y_star(k,:)';Y_star(k+1,:)']-E*Y(k,:,i)');
        tempQs = [q(k,:,i-1)'; q(k+1,:,i-1)'] + I * delta_Q(:,:,k,i);
        q(k,:,i)    = tempQs(1:nInputs);
        q(k+1,:,i)  = tempQs(nInputs+1:end);
        i = i + 1;
        if norm([Y_star(k,:)';Y_star(k+1,:)']-E*Y(k,:,i)')<del
            bExit = true;
        end 
    end
end