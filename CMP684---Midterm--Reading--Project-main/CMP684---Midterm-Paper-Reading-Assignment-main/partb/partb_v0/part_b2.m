% Init.s
bExit = false;
lambda = 0; % vectorize later
nPredictionStep = 2; nIterMax = 10; del = 1e-03;
nInputs = 6; nOutputs = 6;
E = [];
for i=1:nPredictionStep
    E = cat(1, E, eye(nOutputs));
end
for i=1:nPredictionStep
    if i==1
        v_g = [v_g; eye(nInputs)];
    else
        v_g = [v_g; zeros(nInputs)];
    end  
end
gT = v_g';
J(:,:,0,1) = ;
J(:,:,1,1) = ;
%%
for k=1:800 % check traj length
    i = 1;
    while i<=10 || bExit==false
        Y_star_ = [Y_star(k+1,:); Y_star(k+2,:)];
        T_star_ = T_star(k+1);
        A_star_ = T_star_(1:3, 1:3);
        P_star_ = T_star(1:3, 4);
        A_calc(:,:,i,k) = ;
        A_calc_ = A_calc(:,:,i,k);
        D(:,:,i,k+1) = A_star_ * inv(A_calc_);
        D_ = D(:,:,i,k+1);
        Theta(:,:,i,k+1) = acos((D_(1,1) + D_(1,1) + D_(3,3) - 1) / 2);
        Theta_ = Theta(:,:,i,k+1);
        K(:,i,k+1) = [D_(3,2)-D_(2,3); D_(1,3)-D_(3,1); D_(2,1)-D_(1,2)] / (2*sin(Theta_));
        K_ = K(:,i,k+1);
        AngleDiff = K_ * Theta_;
        
        J(:,:,i,k) = ;
        Jk_ = J(:,:,i,k); Jk1_ = J(:,:,i,k+1);
        Phi_ = [inv(Jk_), 0; -inv(Jk1_), inv(Jk1_)]; 
        y_ = ;
        delta_Q_ = inv(Phi_) * (Y_star_ - E * y_);
        q(:,k,i+1) = q(:,k,i) + gT * delta_Q_;
        
        i = i + 1;
        if ()
            bExit = true
        end
    end
   
   
    
    
    
    
end