function err = calculatePredictiveError(y, y_next, q, q_next, angles0, anglesF, Y0, YF, t)
    tf = 10; % sim duration
    A_temp = calcTransform(q);
    A = A_temp(1:3,1:3); % rotation matrix @ current time
    P = A_temp(1:3,4); % position vector @ current time
    %% calc. a_star, b_star, g_star for A_star
    theta0 = angles0; thetaF = anglesF;
    Pi_tf = eul2quat(anglesF');
    Pi_0 = eul2quat(angles0');
    qMult = quatmultiply(quatinv(Pi_0),Pi_tf); % used frequently
    a = acos(qMult(4));
    q1 = qMult(1); q2 = qMult(2); q3= qMult(3); q4 = qMult(4); % for ease
    stf = quatnorm(2*quatmultiply(quatmultiply(Pi_0,quatlog(qMult)), quatinv(Pi_0)));
    a0 = 0; a1 = 0; a2 = 0;
    S1_tf = norm(YF-Y0);
    a3 = (20*S1_tf)/(2*tf^3); a4 = (-30*S1_tf)/(2*tf^4); a5 = (12*S1_tf)/(2*tf^5); 
    b0 = 0; b1 = 0; b2 = 0;
    b3 = (20*stf)/(2*tf^3); b4 = (-30*stf)/(2*tf^4); b5 = (12*stf)/(2*tf^5);
    %% for k=1
    % Traslational:
    st_t = a0+a1*t+a2*t^2+a3*t^3+a4*t^4+a5*t^5;
    p_star = Y0(1:3) + (YF(1:3) - Y0(1:3)) / norm(YF(1:3) - Y0(1:3)) * st_t + Y0(1:3);
   
    st = b0 + b1*t + b2*t^2 + b3*t^3 + b4*t^4 + b5*t^5; % current @ path
    Pi_desired = [ q1*sin(a*st/stf)/sin(a);...
                   q2*sin(a*st/stf)/sin(a);...
                   q3*sin(a*st/stf)/sin(a);...
                      cos(a*st/stf)/sin(a)];
    % Back to euler angles:
    q1_ = Pi_desired(1); q2_ = Pi_desired(2); % for ease
    q3_ = Pi_desired(3); q4_ = Pi_desired(4); % for ease
    Theta = [atan(2*(q4_*q1_+q2_*q3_)/(1-2*(q1_^2+q2_^2)));...
             asin(2*(q4_*q2_-q3_*q1_));...
             atan(2*(q4_*q3_+q1_*q2_)/(1-2*(q2_^2+q3_^2)))];    
    %
    as = Theta(1); bs = Theta(2); gs = Theta(3); % for ease
    T_star = [cos(bs)*cos(gs), cos(gs)*sin(as)*sin(bs)-cos(as)*sin(gs), sin(as)*sin(gs)+cos(as)*cos(gs)*sin(bs);...
              cos(bs)*sin(gs), cos(as)*cos(gs)+sin(as)*sin(gs)*sin(bs), cos(as)*sin(bs)*sin(gs)-cos(gs)*sin(as);...
              -sin(bs), cos(bs)*sin(as), cos(as)*cos(bs)];

    A_star = T_star(1:3,1:3);
    D = A_star * inv(A); % relative
    theta = acos((D(1,1)+D(2,2)+D(3,3)-1)/2);
    K = [D(3,2) - D(2,3);...
         D(1,3) - D(3,1);...
         D(2,1) - D(1,2)]...
         / (2*sin(theta));
    angle_diff = K*theta;

    diff1 = [(p_star(1:3) - P); angle_diff];
    %% for k=2
    % Traslational:
    st_t = a0+a1*(t+1)+a2*(t+1)^2+a3*(t+1)^3+a4*(t+1)^4+a5*(t+1)^5;
    p_star = Y0 + (YF - Y0) / norm(YF - Y0) * st_t + Y0;

    st = b0 + b1*(t+1) + b2*(t+1)^2 + b3*(t+1)^3 + b4*(t+1)^4 + b5*(t+1)^5; % current @ path
    Pi_desired = [ q1*sin(a*st/stf)/sin(a);...
                   q2*sin(a*st/stf)/sin(a);...
                   q3*sin(a*st/stf)/sin(a);...
                      cos(a*st/stf)/sin(a)];
    % Back to euler angles:
    q1_ = Pi_desired(1); q2_ = Pi_desired(2); % for ease
    q3_ = Pi_desired(3); q4_ = Pi_desired(4); % for ease
    Theta = [atan(2*(q4_*q1_+q2_*q3_)/(1-2*(q1_^2+q2_^2)));...
             asin(2*(q4_*q2_-q3_*q1_));...
             atan(2*(q4_*q3_+q1_*q2_)/(1-2*(q2_^2+q3_^2)))];    
    %%
    as = Theta(1); bs = Theta(2); gs = Theta(3); % for ease
    T_star = [cos(bs)*cos(gs), cos(gs)*sin(as)*sin(bs)-cos(as)*sin(gs), sin(as)*sin(gs)+cos(as)*cos(gs)*sin(bs);...
              cos(bs)*sin(gs), cos(as)*cos(gs)+sin(as)*sin(gs)*sin(bs), cos(as)*sin(bs)*sin(gs)-cos(gs)*sin(as);...
              -sin(bs), cos(bs)*sin(as), cos(as)*cos(bs)];

    A_star = T_star(1:3,1:3);
    D = A_star * inv(A); % relative
    theta = acos((D(1,1)+D(2,2)+D(3,3)-1)/2);
    K = [D(3,2) - D(2,3);...
         D(1,3) - D(3,1);...
         D(2,1) - D(1,2)]...
         / (2*sin(theta));
    angle_diff = K*theta;

    diff2 = [(p_star(1:3) - y_next(1:3)'); angle_diff];

    err = [diff1;diff2];
end