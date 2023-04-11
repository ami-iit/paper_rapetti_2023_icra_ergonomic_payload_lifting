function h_b = computeObjectCoriolisAndGravity(M, v_b)

% Coriolis
w_b = v_b(4:6);
C_b = [zeros(3,3), zeros(3,3); zeros(3,3), wbc.skew(w_b) * M(4:6,4:6)] * v_b;

% Gravity
g_b = [0; 0; 9.81 * M(1,1); 0; 0; 0];

% Coriolis + Gravity
h_b = C_b + g_b;

end

