function M = computeBoxMassMatrix(m, size_x, size_y, size_z)

    Ixx = m * (size_y^2+size_z^2) / 12;
    Iyy = m * (size_x^2+size_z^2) / 12;
    Izz = m * (size_x^2+size_y^2) / 12;
    I = [Ixx    0.0    0.0;
         0.0    Iyy    0.0;
         0.0    0.0    Izz];

    M = [   m  0.0 0.0    0.0   0.0     0.0;
            0.0    m 0.0    0.0   0.0   0.0;
            0.0  0.0   m    0.0   0.0   0.0;
            0.0  0.0 0.0  I(1,1)  0.0   0.0;
            0.0  0.0 0.0    0.0 I(2,2)  0.0;
            0.0  0.0 0.0    0.0   0.0 I(3,3)];
    
end

