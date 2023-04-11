function nu_b = computeBaseVelocityFromContacts(J_contact_1, J_contact_2, jointVel, regularization)

    % COMPUTEBASEVELOCITYWITHFEETCONTACT computes the floating base velocity assuming two
    %                                    frames are in contact (they have velocity = 0).
    %
    % FORMAT: nu_b = computeBaseVelocityFromContacts(J_contact_1, J_contact_2, jointVel, regularization)
    %
    % INPUT:  - J_contact_1          = [6 * ROBOT_DOF +6] contact 1 Jacobian
    %         - J_contact_2          = [6 * ROBOT_DOF +6] contact 2 Jacobian
    %         - jointVel             = [ROBOT_DOF * 1] joint velocity
    %         - regularization       = tolerance for matrix pseudoinverse.
    %
    % OUTPUT: - nu_b              = [6 * 1] floating base velocity
    %
    % Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava, Lorenzo Rapetti
    %          
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, 2021
    %
 
    %% --- Initialization ---

    % Compute full contacts Jacobian
    Jc     = [J_contact_1;
              J_contact_2];

    % Compute multiplier of nu_b  
    pinvJb = wbc.pinvDamped(Jc(:,1:6),regularization);
  
    % Base velocity
    nu_b   = -pinvJb*Jc(:,7:end)*jointVel;
end