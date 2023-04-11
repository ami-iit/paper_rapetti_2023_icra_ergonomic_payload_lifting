function JDot_nu = computeContactsJDotNu(w_H, handlings, v_b)

w_R_box = w_H(1:3, 1:3);
b_w_b   = v_b(4:6);

w_R_box_dot = w_R_box * wbc.skew(b_w_b);

JDot_l_hand_1 = [zeros(3,3), - w_R_box_dot * wbc.skew(handlings(1,:)');
                 zeros(3,3), w_R_box_dot];
JDot_r_hand_1 = [zeros(3,3), - w_R_box_dot * wbc.skew(handlings(2,:)');
                 zeros(3,3), w_R_box_dot];
JDot_l_hand_2 = [zeros(3,3), - w_R_box_dot * wbc.skew(handlings(3,:)');
                 zeros(3,3), w_R_box_dot];
JDot_r_hand_2 = [zeros(3,3), - w_R_box_dot * wbc.skew(handlings(4,:)');
                 zeros(3,3), w_R_box_dot];
             
JDot_nu = [JDot_l_hand_1; JDot_r_hand_1; JDot_l_hand_2; JDot_r_hand_2] * v_b;

end

