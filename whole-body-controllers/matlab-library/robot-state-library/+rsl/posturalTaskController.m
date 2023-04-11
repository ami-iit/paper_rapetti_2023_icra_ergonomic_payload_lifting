function [tau_0_model, tau_0_sigma] =  posturalTaskController(jointPos, nu, J, M, h, jointPos_des, jointVel_des, GainPostural)

    % The mass matrix is partitioned as:
    %
    %   M = [ Mb,   Mbs
    %         Mbs', Ms ];  
    %
    % where: Mb  \in R^{6 x 6}
    %        Mbs \in R^{6 x 6+NDOF}
    %        Ms  \in R^{NDOF x NDOF}
    %
    Mb          = M(1:6,1:6);
    Mbs         = M(1:6,7:end);
    Ms          = M(7:end,7:end);
    
    Mbar        = Ms-Mbs'/Mb*Mbs;

    % Joints velocity and joints position error
    jointVel        = nu(7:end);
    jointPosTilde   = jointPos - jointPos_des;
    jointVelTilde   = jointVel - jointVel_des;

    % Get the vector tauModel
    u_0             = - GainPostural.Kp*jointPosTilde - GainPostural.Kd*jointVelTilde;
    tau_0_model     = (h(7:end) - Mbs'/Mb*h(1:6) + Mbar * u_0); 
    tau_0_sigma     = - transpose(J(:,7:end)) + Mbs'/Mb*transpose(J(:,1:6));
        

end