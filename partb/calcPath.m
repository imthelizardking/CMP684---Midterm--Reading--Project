function pose_star = calcPath(pose_init, pose_final, q_init, q_final,t)
    [p_star,~] = s1(t, q_init, q_final);

    theta_init = pose_init(4:6)';
    theta_final = pose_final(4:6)';

    sa_tf = sin(pose_final(4)/2); ca_tf = cos(pose_final(4)/2);
    sb_tf = sin(pose_final(5)/2); cb_tf = cos(pose_final(5)/2);
    sg_tf = sin(pose_final(6)/2); cg_tf = cos(pose_final(6)/2);

%     Quat_tf = [sa_tf*cb_tf*cg_tf-ca_tf*sb_tf*sg_tf;...
%                ca_tf*sb_tf*cg_tf+sa_tf*cb_tf*sg_tf;...
%                ca_tf*cb_tf*sg_tf+sa_tf*sb_tf*sg_tf;...
%                ca_tf*cb_tf*cg_tf+sa_tf*sb_tf*sg_tf];
    Quat_tf = eul2quat(pose_final(4:6)');

    sa_t0 = sin(pose_init(4)/2); ca_t0 = cos(pose_init(4)/2);
    sb_t0 = sin(pose_init(5)/2); cb_t0 = cos(pose_init(5)/2);
    sg_t0 = sin(pose_init(6)/2); cg_t0 = cos(pose_init(6)/2);

%     Quat_t0 = [sa_t0*cb_t0*cg_t0-ca_t0*sb_t0*sg_t0;...
%                ca_t0*sb_t0*cg_t0+sa_t0*cb_t0*sg_t0;...
%                ca_t0*cb_t0*sg_t0+sa_t0*sb_t0*sg_t0;...
%                ca_t0*cb_t0*cg_t0+sa_t0*sb_t0*sg_t0];
    Quat_t0 = eul2quat(pose_init(4:6)');

    S2_tf = norm(quatmultiply(quatmultiply(2*Quat_t0,log(quatmultiply(quatinv(Quat_t0),Quat_tf))),quatinv(Quat_t0)));
    [angle_Vec,S2_t] = s2(t, pose_init, pose_final);

    Quat_bar = (quatmultiply(quatinv(Quat_t0),Quat_tf))'; % (*44)
    a_ = acos(Quat_bar(4)); % (*45)
    temp1 = a_*S2_t/S2_tf;
    Quat_k = quatmultiply(Quat_t0,[Quat_bar(1)*sin(temp1)/sin(a_),... % (*46,47)
              Quat_bar(2)*sin(temp1)/sin(a_),...
              Quat_bar(3)*sin(temp1)/sin(a_),...
              cos(temp1)]);
    Theta_k = [atan(2*(Quat_k(4)*Quat_k(1)+Quat_k(2)*Quat_k(3))/(1-2*(Quat_k(1)^2+Quat_k(2)^2)));...
               asin(2*(Quat_k(4)*Quat_k(2)-Quat_k(3)*Quat_k(1)));...
               atan(2*(Quat_k(4)*Quat_k(3)+Quat_k(1)*Quat_k(2))/(1-2*(Quat_k(2)^2+Quat_k(3)^2)))];

    pose_star = [p_star;Theta_k];
end