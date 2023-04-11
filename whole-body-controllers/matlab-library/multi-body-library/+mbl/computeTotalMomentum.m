function L = computeTotalMomentum(L_1, L_2, L_B, pos_CoM_1, pos_CoM_2, pos_CoM_B, pos_CoM_total)

L_linear = L_1(1:3) +  L_2(1:3) +  L_B(1:3);

L_angular = L_1(4:6) + L_2(4:6) + L_B(4:6) + ...
            wbc.skew(pos_CoM_1-pos_CoM_total) * L_1(1:3) + ...
            wbc.skew(pos_CoM_2-pos_CoM_total) * L_2(1:3) + ...
            wbc.skew(pos_CoM_B-pos_CoM_total) * L_B(1:3);

L = [L_linear; L_angular];

end