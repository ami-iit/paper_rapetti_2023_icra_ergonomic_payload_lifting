function [wrench_Aeq, wrench_beq, wrench_Adeq, wrench_bdeq] =  momentumControllerWrench(L, pos_CoM, desired_pos_vel_acc_CoM, w_H_meas, w_R_des, w_H_contacts, mass, constraintsMatrixA, constraintsVectorb, linksUseConstraintMatrix, GainMomentum)

% get the number of contacts from the contact transform matrix
number_of_contacts = size(w_H_contacts,1) / 4;

%% construct the Aeq matrix
wrench_Aeq = zeros(6, 6*number_of_contacts);
for i=1:number_of_contacts
    % A_i is the wrench transform from the contact to the CoM
    w_H_i = w_H_contacts((1+(i-1)*4):(i*4), :);
    r_i   = w_H_i(1:3,4) - pos_CoM;
    A_i = [eye(3),        zeros(3);
           wbc.skew(r_i),   eye(3)];
       
    wrench_Aeq(1:6,(1+(i-1)*6):(i*6)) = A_i;
end

%% construct the beq vector
% containing the desired Momentum derivative and the gravity term
vel_CoM     = L(1:3)/mass;
acc_CoM_star = desired_pos_vel_acc_CoM(:,3) - GainMomentum.Kp(1:3) .* (pos_CoM - desired_pos_vel_acc_CoM(:,1)) - GainMomentum.Kd(1:3) .* (vel_CoM - desired_pos_vel_acc_CoM(:,2));
LDot_star = [mass * acc_CoM_star;
             -GainMomentum.Kp(4:6) .* L(4:6) - GainMomentum.Ki(4:6) .*  wbc.skewVee(w_H_meas(1:3,1:3) * w_R_des)];
f_grav    = [zeros(2,1); - mass * 9.81; zeros(3,1)];

wrench_beq = LDot_star - f_grav;

%% construct constraints matrix
sizeConstraintMatrix = size(constraintsMatrixA, 1);
wrench_Adeq = zeros(number_of_contacts * sizeConstraintMatrix, 6*number_of_contacts);
wrench_bdeq = zeros(number_of_contacts * sizeConstraintMatrix,1);

idxConstr = 0;
for idxContacts=1:number_of_contacts
    if (linksUseConstraintMatrix(idxContacts))
        w_R_contacts_i = w_H_contacts((1+(idxContacts-1)*4):(3+(idxContacts-1)*4),1:3);
        wrench_Adeq((idxConstr + 1):(idxConstr+sizeConstraintMatrix), (1+(idxContacts-1)*6):idxContacts*6 ) = constraintsMatrixA * blkdiag(w_R_contacts_i', w_R_contacts_i');
        wrench_bdeq((idxConstr + 1):(idxConstr+sizeConstraintMatrix),1) = constraintsVectorb;
    end
    idxConstr = idxConstr + sizeConstraintMatrix;
end


end
