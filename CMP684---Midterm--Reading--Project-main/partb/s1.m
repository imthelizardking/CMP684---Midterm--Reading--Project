function [pos_vec, S1_t] = s1(t, q_init, q_final)
    tf = 10; % = k_end
    temp_p_star_tf = (calcTransform(q_final));
    p_star_tf = temp_p_star_tf(1:3,4);
    temp_p_star_0 = (calcTransform(q_init));
    p_star_0 = temp_p_star_0(1:3,4);    
    S1_tf = norm(p_star_tf-p_star_0);
    a0 = 0;
    a1 = 0;
    a2 = 0;
    a3 = (20*S1_tf)/(2*tf^3);
    a4 = (-30*S1_tf)/(2*tf^4);
    a5 = (12*S1_tf)/(2*tf^5);
    S1_t = a0+a1*t+a2*t^2+a3*t^3+a4*t^4+a5*t^5;
    pos_vec = a0 + (p_star_tf - p_star_0) / norm(p_star_tf - p_star_0) * S1_t + a0;
end