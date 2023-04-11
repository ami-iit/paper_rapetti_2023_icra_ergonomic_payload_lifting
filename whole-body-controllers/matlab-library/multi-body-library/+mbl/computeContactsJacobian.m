function J = computeContactsJacobian(w_H, contacts)

w_R = w_H(1:3, 1:3);

J_l_hand_1 = [     eye(3),            -w_R * wbc.skew(contacts(1,:)');
               zeros(3,3),                                        w_R];
J_r_hand_1 = [     eye(3),            -w_R * wbc.skew(contacts(2,:)');
               zeros(3,3),                                        w_R];
J_l_hand_2 = [     eye(3),            -w_R * wbc.skew(contacts(3,:)');
               zeros(3,3),                                        w_R];
J_r_hand_2 = [     eye(3),            -w_R * wbc.skew(contacts(4,:)');
               zeros(3,3),                                        w_R];
           
J = [J_l_hand_1; J_r_hand_1; J_l_hand_2; J_r_hand_2];

end

