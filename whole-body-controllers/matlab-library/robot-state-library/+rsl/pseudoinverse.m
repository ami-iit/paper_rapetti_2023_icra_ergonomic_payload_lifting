function [yMinNorm, null] = pseudoinverse(Aeq, beq, InvTol)

% pseudoinverse solution
pinvA = pinv(Aeq, InvTol);
yMinNorm = pinvA * beq;

% compute the null space for the force
null = (eye(size(pinvA,1), size(pinvA,1)) - pinvA*Aeq);

end
