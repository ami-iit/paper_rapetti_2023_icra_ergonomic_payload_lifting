function M = concatenateMassMatrices(M_1, M_2, M_B)

M = blkdiag(M_1, M_2, M_B);

end