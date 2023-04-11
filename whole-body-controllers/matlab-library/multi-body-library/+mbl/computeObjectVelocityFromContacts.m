function v_b = computeObjectVelocityFromContacts(v_1, v_2, w_H_1, w_H_2, w_H_B)

w_v_linear_l_hand_1 = v_1(13:15);
w_v_linear_r_hand_1 = v_1(19:21);
w_v_linear_l_hand_2 = v_2(13:15); 
w_v_linear_r_hand_2 = v_2(19:21);

w_p_l_hand_1 = w_H_1(9:11, 4);
w_p_r_hand_1 = w_H_1(13:15, 4);
w_p_l_hand_2 = w_H_2(9:11, 4);
w_p_r_hand_2 = w_H_2(13:15, 4);

w_x_axisDot_box = mbl.computeVersorDerivate(w_p_l_hand_1, w_p_r_hand_2, w_v_linear_l_hand_1, w_v_linear_r_hand_2);
w_y_axisDot_box = mbl.computeVersorDerivate(w_p_r_hand_1, w_p_l_hand_1, w_v_linear_r_hand_1, w_v_linear_l_hand_1);
%w_x_axisDot_box = computeVersorDerivate(w_p_l_hand_1, w_p_r_hand_1, w_v_linear_l_hand_1, w_v_linear_r_hand_1);
%w_y_axisDot_box = computeVersorDerivate(w_p_l_hand_1, w_p_r_hand_2, w_v_linear_l_hand_1, w_v_linear_r_hand_2);
w_z_axisDot_box = wbc.skew(w_x_axisDot_box) * w_H_B(1:3,2) + wbc.skew(w_H_B(1:3,1)) * w_y_axisDot_box;

w_RDot_B = [w_x_axisDot_box, w_y_axisDot_box, w_z_axisDot_box];

B_omega_B = wbc.skewVee( w_H_B(1:3, 1:3)' * w_RDot_B);

w_v_B = (w_v_linear_l_hand_1 + w_v_linear_r_hand_1 + w_v_linear_l_hand_2 + w_v_linear_r_hand_2) / 4;

v_b   = [w_v_B; B_omega_B];

end

