clear all; clc; close all;
%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  % % % % % % % % % % % % % % % % % % % % % % %
% In this script, "V. Simulation", "Example 1" part of the paper "iscussions on Inverse Kinematics based on Levenberg-Marquardt Method and Model-Free
% Adaptive (Predictive) Control" is implemented.
% Yusuf Çağrı Öğüt
% CMP684-Neural Networks, Fall 2022
% Hacettepe University, Computer Engineering
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%% Parameters
nLength = 800; % number of simulation steps.
nInputs = 3; nOutputs = 3; % Number of inputs & outputs
nPredictionStep = 5; % Horizon length for Predictive Control
l1 = 5; l2 = 7; l3 = 7; % robot geometric parameters (lengths of robot links)
vK = 1:800; % Sim. vector
x_star = 4+3*sin(pi*vK/50); % Reference trajectory, x component (Given after Eq. (43) in the paper)
y_star = 3*cos(pi*vK/50); % Reference trajectory, y component (Given after Eq. (43) in the paper)
z_star = 5 + vK/200; % Reference trajectory, z component (Given after Eq. (43) in the paper)
Y_star = [x_star', y_star', z_star']; % Reference Trajectory, as matrix
%% Initializations
I = eye(nPredictionStep*nOutputs); % Identity Matrix, size[nPredictionStep x nOutputs),(nPredictionStep x nOutputs)]
v_g = []; % Multiplier matrix used in equations (Defined in after Eqn. (28) in the paper)
for i=1:nPredictionStep
    if i==1
        v_g = [v_g; eye(nInputs)];
    else
        v_g = [v_g; zeros(nInputs)];
    end  
end
gT = v_g';
E = []; % Multiplier matrix used in equations (Defined in after Eqn. (19) in the paper)
for i=1:nPredictionStep
    E = cat(1, E, eye(nOutputs));
end
lambda = zeros(nLength, 1); lambda(1) = 2; lambda(2) = 2; % Control parameter (usage defined under Eqn. (44)), lambda, initiated w/ "2"
delta_u = zeros(nLength, nInputs); % "Change of input" vector, size [nLength, nInputs] Eqn. (44)
phi = zeros(nPredictionStep*nInputs, nPredictionStep*nInputs, nLength); % Phi matrix, Defined in after Eqn. (44)
J = zeros(nInputs, nOutputs,nLength); % Jacobian matrix, derived from given kinematic relations at Eqn. (42)
q1 = zeros(nLength,1); q2 = zeros(nLength,1); q3 = zeros(nLength,1); % Joint angle vectors q1, q2 and q3
q1(1) = 0; q2(1) = 0; q3(1) = 0; % Initilization
x = zeros(nLength,1); y = zeros(nLength,1); z = zeros(nLength,1); % Task space variables x, y and z
x(1) = 0; y(1) = 0; z(1) = 0; % Initilization
Y = zeros(nOutputs, nLength); % Task space matrix
Y(:,1) = [x(1); x(2); x(3)]; % Initilization
Y_horizon = zeros(nOutputs*nPredictionStep,1); % Horizon vector for Predictive Control
for k=2:(nLength-10) % 1st time step is already initialized and due to predictive control, Time Vector is cropped from the end.
    %% Precalculations, Jacobian and Phi:
    % Jacobian calculation from previous time step joint angles and kinematic relations (42)
    J(:,:,k) = [-l3*sin(q2(k-1)+q3(k-1))*sin(q1(k-1)), l2*cos(q2(k-1))+l3*cos(q2(k-1)+q3(k-1))*cos(q1(k-1)), l3*cos(q2(k-1)+q3(k-1))*cos(q1(k-1)); ...
                             l3*sin(q2(k-1)+q3(k-1))*cos(q1(k-1)), l2*cos(q2(k-1))+l3*cos(q2(k-1)+q3(k-1))*sin(q1(k-1)),            l3*cos(q2(k-1)+q3(k-1))*sin(q1(k-1));...
                             0                                   ,-l2*sin(q2(k-1))-l3*sin(q2(k-1)+q3(k-1))            ,            -l3*sin(q2(k-1)+q3(k-1))];
    % Phi matrix w/ local approximation:
    for i=1:nPredictionStep
        phi(1:3,   3*(i-1)+1:3*(i-1)+1+2,k) = J(:,:,k);
        phi(4:6,   3*(i-1)+1:3*(i-1)+1+2,k) = J(:,:,k);
        phi(7:9,   3*(i-1)+1:3*(i-1)+1+2,k) = J(:,:,k);
        phi(10:12, 3*(i-1)+1:3*(i-1)+1+2,k) = J(:,:,k);
        phi(13:15, 3*(i-1)+1:3*(i-1)+1+2,k) = J(:,:,k);
    end    
%% Apply control law in (44) to find required amount of input change, delta_u(k) :
    Y_star_current = [Y_star(k+1,:)'; Y_star(k+2,:)'; Y_star(k+3,:)'; Y_star(k+4,:)'; Y_star(k+5,:)']; % Current horizon, reference trajectory
    delta_u(k,:) = gT * inv(phi(:,:,k)'*phi(:,:,k)+lambda(k)*I)*phi(:,:,k)'...
                        *(Y_star_current-E*Y(:,k));
    q1(k) = q1(k-1) + delta_u(k,1); % Current q1 joint angle 
    q2(k) = q2(k-1) + delta_u(k,2); % Current q2 joint angle
    q3(k) = q3(k-1) + delta_u(k,3); % Current q3 joint angle
%% Find Y(k+1) w/ Equation (24), next time step Task Space variables:
    Y(:,k+1) = Y(:,k) + J(:,:,k) * delta_u(k,:)'; % Task Space variables
    x(k+1) = Y(1,k+1); y(k+1) = Y(2,k+1); z(k+1) = Y(3,k+1); % Task Space variables
%% Find next "nPredictionStep" Task Space Variables, horizon:
    Y_horizon(1:3) = Y(:, k+1); % First set of task space variables are already calculated above
    q1_temp = q1(k); q2_temp = q2(k); q3_temp = q3(k); % Temporary joint angles are assigned
    Y_temp = Y(:,k+1); % Temporary Task Space matrix is assigned
    for i=1:nPredictionStep-1 % get 4:6 | 7:9 | 10:12 | 13:15        
        for ii=1:nPredictionStep
            phi_temp(1:3,   3*(ii-1)+1:3*(ii-1)+1+2) = J(:,:,k); % Local approximation made,i.e. J is same for entire horizon
            phi_temp(4:6,   3*(ii-1)+1:3*(ii-1)+1+2) = J(:,:,k);
            phi_temp(7:9,   3*(ii-1)+1:3*(ii-1)+1+2) = J(:,:,k);
            phi_temp(10:12, 3*(ii-1)+1:3*(ii-1)+1+2) = J(:,:,k);
            phi_temp(13:15, 3*(ii-1)+1:3*(ii-1)+1+2) = J(:,:,k);
        end
        % Next "nPredictionStep" reference trajectory values:
        Y_horizon_star_temp = [Y_star(k+i+1,:)'; Y_star(k+i+2,:)'; Y_star(k+i+3,:)'; Y_star(k+i+4,:)'; Y_star(k+i+5,:)'];
        % Required amount of change in input (joint angles), for current prediction step:
        delta_u_temp = gT * inv(phi_temp'*phi_temp+lambda(k)*I)*phi_temp'...
                            *(Y_horizon_star_temp-E*Y_temp);
        q1_temp = q1_temp + J(:,:,k)*delta_u_temp(1); % temporary joint angle q1
        q2_temp = q2_temp + J(:,:,k)*delta_u_temp(2); % temporary joint angle q2
        q3_temp = q3_temp + J(:,:,k)*delta_u_temp(3); % temporary joint angle q3
        Y_temp = Y_temp + J(:,:,k)*delta_u_temp; % temporary task space variables
        Y_horizon(i*3+1:i*3+3) = Y_temp; % Horizon Task Space variables
    end   
    % Update Lambda (Controller Parameter) using Eqn. (44):
    Y_horizon_star = []; % Init Reference Trajectory Horizon vector
    % Set Reference Trajectory Horizon vector for condition in Eqn. (44):
    for i=1:nPredictionStep
        Y_horizon_star = [Y_horizon_star; Y_star(k+i,:)'];
    end
    % Check if error for next "nPredictionStep" is bigger than the threshold
    if norm(Y_horizon_star - Y_horizon)>10
        lambda(k+1) = 1.1*lambda(k); % if so, increase lambda
    else
        lambda(k+1) = lambda(k)/1.02; % if not, decrease lambda
    end
end
%% Continue for plottings:
%% Trajectory (3d point cloud):
scatter3(x,y,z,'r','filled');
hold on
scatter3(x_star,y_star,z_star,'b','filled');
%% Trajectory:
plot(x,'DisplayName','x');hold on;plot(x_star,'DisplayName','x_star');plot(y,'DisplayName','y');plot(y_star,'DisplayName','y_star');plot(z,'DisplayName','z');plot(z_star,'DisplayName','z_star');hold off;
grid minor
legend
%% Plot q's (joint angles):
plot(q1,'DisplayName','q1');hold on;plot(q2,'DisplayName','q2');plot(q3,'DisplayName','q3');hold off;
grid minor
legend
%% Plot lambda:
plot(lambda)
grid minor
legend
%% Jacobian elements:
j11 = reshape(J(1,1,:),[800,1]);
j12 = reshape(J(1,2,:),[800,1]);
j13 = reshape(J(1,3,:),[800,1]);
j21 = reshape(J(2,1,:),[800,1]);
j22 = reshape(J(2,2,:),[800,1]);
j23 = reshape(J(2,3,:),[800,1]);
j31 = reshape(J(3,1,:),[800,1]);
j32 = reshape(J(3,2,:),[800,1]);
j33 = reshape(J(3,3,:),[800,1]);
plot(j11,'DisplayName','j11');hold on;plot(j12,'DisplayName','j12');plot(j13,'DisplayName','j13');plot(j21,'DisplayName','j21');plot(j22,'DisplayName','j22');plot(j23,'DisplayName','j23');plot(j31,'DisplayName','j31');plot(j32,'DisplayName','j32');plot(j33,'DisplayName','j33');hold off;
grid minor
legend
%%