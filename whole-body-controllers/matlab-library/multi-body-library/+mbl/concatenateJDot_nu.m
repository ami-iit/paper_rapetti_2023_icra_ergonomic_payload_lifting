function JDot_nu = concatenateJDot_nu(JDot_nu_1, JDot_nu_2, JDot_nu_B)

JDot_nu = [JDot_nu_1; zeros(24,1)] + [zeros(24,1); JDot_nu_2] + [zeros(12,1); -JDot_nu_B(1:12,1); zeros(12,1); -JDot_nu_B(13:24,1)];

end