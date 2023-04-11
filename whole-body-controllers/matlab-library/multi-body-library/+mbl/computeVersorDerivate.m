function w_axis_dot = computeVersorDerivate(w_p_1,w_p_2, w_v_linear_1, w_v_linear_2)

pos_diff  = w_p_2 - w_p_1;
vel_diff  = w_v_linear_2 - w_v_linear_1;
norm_diff = norm(pos_diff);

w_axis_dot = ( vel_diff * norm_diff - pos_diff * pos_diff' * vel_diff / norm_diff ) / norm_diff^2;

end

