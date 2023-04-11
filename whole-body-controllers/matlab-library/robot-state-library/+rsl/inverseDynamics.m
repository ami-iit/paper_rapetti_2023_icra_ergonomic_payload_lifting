function [tau_model, tau_sigma, LambdaAugmented] = inverseDynamics(tau_0_model, tau_0_sigma, J, JDot_nu, M, h, B, fixedLinkVelcityComponents, Reg)

LambdaAugmented = zeros(size(J,1), length(tau_0_model));
idxLinkVelocityComponents = find(fixedLinkVelcityComponents);

% invert projected dynamics
J_invM = J(idxLinkVelocityComponents,:)/M;
Lambda = J_invM*B;
pinvLambda = wbc.pinvDamped(Lambda, Reg.LambdaInvDump);
NullLambda  =  eye(size(pinvLambda,1), size(pinvLambda, 1)) - pinvLambda*Lambda;

% compute wrench multiplier Sigma
tau_sigma = -pinvLambda*J_invM*J' + NullLambda*(tau_0_sigma);

% compute tau_model, which is the torque assuming f=0
tau_model = pinvLambda*(J_invM*h - JDot_nu(idxLinkVelocityComponents)) + NullLambda*tau_0_model;

LambdaAugmented(1:size(Lambda,1), :) = Lambda; 

end