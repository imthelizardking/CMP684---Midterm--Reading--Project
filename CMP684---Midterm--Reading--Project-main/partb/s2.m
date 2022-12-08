function [angle_vec,S1_t] = s2(t, pose_init, pose_final)
    tf = 10; % = k_end
    a_star_tf = pose_init(4:6);
    a_star_t0 = pose_final(4:6);   
    S1_tf = norm(a_star_tf-a_star_t0);
    b0 = 0;
    b1 = 0;
    b2 = 0;
    b3 = (20*S1_tf)/(2*tf^3);
    b4 = (-30*S1_tf)/(2*tf^4);
    b5 = (12*S1_tf)/(2*tf^5);
    S1_t = b0+b1*t+b2*t^2+b3*t^3+b4*t^4+b5*t^5;
    angle_vec = b0 + (a_star_tf - a_star_t0) / norm(a_star_tf - a_star_t0) * S1_t + b0;
end