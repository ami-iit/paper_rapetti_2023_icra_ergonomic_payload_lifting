function L_B = computeObjectMomentumInWorld(M, v_B, w_H)

w_R = w_H(1:3,1:3);
L_B = [eye(3,3), zeros(3,3); zeros(3,3), w_R] * M * v_B;

end

