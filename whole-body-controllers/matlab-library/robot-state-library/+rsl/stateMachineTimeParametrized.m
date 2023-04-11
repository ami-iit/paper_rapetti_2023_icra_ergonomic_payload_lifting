function [jointPos_des, jointVel_des, posCoM_des, useForwardKinematicsForCoM, phiOut, currentStateOut] = stateMachineTimeParametrized(jointPos, nu, posCoM, time, J, linkPose, linkPoseDes, linkVelocityDes, configStateMachine)

persistent currentState;
persistent t0;
persistent phi;
persistent jointPose_des_old;
persistent time_old;

persistent linkPoseError_zero;

if isempty(currentState)
    currentState = 1;
end
if isempty(t0)
    t0 = time;
end
if isempty(phi)
    phi = 0 * jointPos;
end
if isempty(jointPose_des_old)
    jointPose_des_old = 0 * jointPos;
end



if isempty(time_old)
    time_old = time;
end

% Update state
if (time-t0 > configStateMachine.tEnd(currentState))
    if (and (configStateMachine.loop, (time-t0) > configStateMachine.tEnd(end)) )
        currentState = 1;
        t0 = time;
    else
        currentState = currentState + 1;
        phi = 0 * jointPos;
    end
end

stateType = configStateMachine.stateType(currentState);

Deltat = time - time_old;
    
%% STATE 2: CONFIGURATION STATE
if stateType == 2 
    posCoM_des = configStateMachine.posCoM(currentState, :)';
    useForwardKinematicsForCoM = configStateMachine.useForwardKinematicsForCoM(currentState);
    
    jointPos_des = configStateMachine.jointPos(currentState,:)';
    jointVel_des = nu(7:end);
    jointPose_des_old = jointPos;
    
    % Initialize variables for QP
    
    phi = 0 * jointPos;

%% STATE 3: TIME PARAMETRIZED
elseif stateType == 3 
    posCoM_des = configStateMachine.posCoM(currentState, :)';
    useForwardKinematicsForCoM = configStateMachine.useForwardKinematicsForCoM(currentState);
    
    jointPose_delta = configStateMachine.jointPos(currentState,:)'-jointPose_des_old;
    
    
    idxFixedLinkVelocityComponents = find(configStateMachine.fixedLinkVelcityComponents(currentState, :));
    idxControlledLinkVelocityComponents = find(configStateMachine.controlledLinkVelcityComponents(currentState, :));

    % linkPose(:,4)
    % linkPoseDes(:,4)
    % idxControlledLinkVelocityComponents

    if isempty(linkPoseError_zero)
        linkPoseError_zero = [linkPose(1:3,4) - linkPoseDes(1:3,4);
                   zeros(3,1);                           % TODO rotational error not implemented
                   linkPose(5:7,4) - linkPoseDes(5:7,4);
                   zeros(3,1);                           % TODO rotational error not implemented
                   linkPose(9:11,4) - linkPoseDes(9:11,4);
                   zeros(3,1);                           % TODO rotational error not implemented
                   linkPose(13:15,4) - linkPoseDes(13:15,4);
                   zeros(3,1)];
    end
    
    errorVector = [linkPose(1:3,4) - linkPoseDes(1:3,4);
                   zeros(3,1);                           % TODO rotational error not implemented
                   linkPose(5:7,4) - linkPoseDes(5:7,4);
                   zeros(3,1);                           % TODO rotational error not implemented
                   linkPose(9:11,4) - linkPoseDes(9:11,4);
                   zeros(3,1);                           % TODO rotational error not implemented
                   linkPose(13:15,4) - linkPoseDes(13:15,4);
                   zeros(3,1)] - linkPoseError_zero;
               % TODO rotational error not implemented
    frameDes = [zeros(length(idxFixedLinkVelocityComponents),1);
                -configStateMachine.controlledLinksKp(currentState) * errorVector(idxControlledLinkVelocityComponents) - linkVelocityDes(idxControlledLinkVelocityComponents)];
    J_sel = [J(idxFixedLinkVelocityComponents,:)
             J(idxControlledLinkVelocityComponents,:)];
    
    Jsel_deltaPos = J_sel(:,:) * [configStateMachine.baseDelta(currentState,:)'; jointPose_delta];
    
    % solve as pinv  
    pinvJsel_deltaPos = pinv(Jsel_deltaPos);
    phiDot = pinvJsel_deltaPos * frameDes;

    % Null = eye(size(pinvJsel_deltaPos, 1)) - pinvJsel_deltaPos * Jsel_deltaPos;
    % phiDot = pinvJsel_deltaPos * frameDes + Null * (ones(n,1)-phi);
    

    % treshold on DeltaPhi
    DeltaPhi = phiDot*Deltat;
    DeltaPhi(find(DeltaPhi >  configStateMachine.DeltaPhiMax(currentState))) =  configStateMachine.DeltaPhiMax(currentState);
    DeltaPhi(find(DeltaPhi < -configStateMachine.DeltaPhiMax(currentState))) = -configStateMachine.DeltaPhiMax(currentState);
    
    phi = phi + DeltaPhi;
    
    % threshould to have 0=<phi<=1
    phi(find(phi < 0)) = 0;
    phi(find(phi > 1)) = 1;
    
    jointPos_des = jointPose_des_old + jointPose_delta .* phi;
    jointVel_des = jointPose_delta .* phiDot;
    
%% STATE 1: MEASURED STATE
else
    jointPos_des = jointPos;
    jointVel_des = nu(7:end);
    posCoM_des = posCoM;
    useForwardKinematicsForCoM = configStateMachine.useForwardKinematicsForCoM(currentState);
    
    jointPose_des_old = jointPos;

    phi = 0 * jointPos;
end
    
currentStateOut = currentState;

time_old = time;

phiOut = phi;

end
