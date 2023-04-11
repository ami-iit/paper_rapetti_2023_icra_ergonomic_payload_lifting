function J = concatenateJacobians(J_1, J_2, J_B)

J = [ [J_1; zeros(24,size(J_1,2))], [zeros(24,size(J_2,2)); J_2], [zeros(12,6); -J_B(1:12,:); zeros(12,6); -J_B(13:24,:)] ];

end
