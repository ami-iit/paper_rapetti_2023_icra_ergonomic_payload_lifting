function [H, g, A, ubA] = prepareQPOptimization(yMinNorm, null, tau_model, tau_sigma, Adeq, bdeq, yDes, regularizationHessian, regularizationYDes, k_tau_minimization)

% weight matrix
W = diag(k_tau_minimization.^2);

% prepare Hessian and gradient
tau_sigma_Null = tau_sigma * null;
H = tau_sigma_Null' * W * tau_sigma_Null + null' * (regularizationYDes) * null + regularizationHessian;
g = tau_sigma_Null' * W * (tau_model + tau_sigma * yMinNorm) + null' * (regularizationYDes) * (yMinNorm - yDes);

% prepare constraints
A = Adeq * null;
ubA = bdeq - Adeq * yMinNorm;

end