function w_H_b = computeObjectPoseFromContacts(w_H_1, w_H_2, removeInitialzOffsetOnZ, useOnly2ForDesPos)

persistent l_hand_2_offset
persistent r_hand_2_offset

% extract handlings position
w_p_l_hand_1 = w_H_1(9:11, 4);
w_p_r_hand_1 = w_H_1(13:15, 4);
w_p_l_hand_2 = w_H_2(9:11, 4);
w_p_r_hand_2 = w_H_2(13:15, 4);

if (removeInitialzOffsetOnZ)
    if isempty(l_hand_2_offset)
        l_hand_2_offset = w_p_r_hand_1(3) - w_p_l_hand_2(3);
    end
    if isempty(r_hand_2_offset)
        r_hand_2_offset = w_p_l_hand_1(3) - w_p_r_hand_2(3);
    end

    w_p_l_hand_2(3) = w_p_l_hand_2(3) + l_hand_2_offset;
    w_p_r_hand_2(3) = w_p_r_hand_2(3) + r_hand_2_offset;
end



w_x_axis_box = (w_p_r_hand_2-w_p_l_hand_1) / norm(w_p_r_hand_2-w_p_l_hand_1);
w_y_axis_box = (w_p_l_hand_1-w_p_r_hand_1) / norm(w_p_l_hand_1-w_p_r_hand_1);
w_z_axis_box = cross(w_x_axis_box, w_y_axis_box);

w_R_b = [w_x_axis_box, w_y_axis_box, w_z_axis_box];
w_p_B = (w_p_l_hand_1 + w_p_r_hand_1 + w_p_r_hand_2 + w_p_l_hand_2) / 4;

if(useOnly2ForDesPos)
    w_p_B(3) = (w_p_r_hand_2(3) + w_p_l_hand_2(3)) / 2;
end


w_H_b = [w_R_b, w_p_B;
         0 0 0,     1];

end

