function [jointPos_des, posCoM_des, smoothingTime, useSmoothing, useForwardKinematicsForCoM, currentStateOut] = stateMachine(jointPos, posCoM, time, configStateMachine)

persistent currentState;
persistent t0;

if isempty(currentState)
    currentState = 1;
end
if isempty(t0)
    t0 = time;
end

% Update state
if (time-t0 > configStateMachine.tEnd(currentState))
    if (and (configStateMachine.loop, (time-t0) > configStateMachine.tEnd(end)) )
        currentState = 1;
        t0 = time;
    else
        currentState = currentState + 1;
    end
end

stateType = configStateMachine.stateType(currentState);

%% STATE 1: MEASURED STATE
jointPos_des = jointPos;
posCoM_des = posCoM;
smoothingTime = configStateMachine.smoothingTime(currentState);
useSmoothing = configStateMachine.useSmoothing(currentState);
useForwardKinematicsForCoM = configStateMachine.useForwardKinematicsForCoM(currentState);

%% STATE 2,3: CONFIGURATION STATE
if or(stateType == 2, stateType == 3) 
    
    jointPos_des = configStateMachine.jointPos(currentState,:)';
    posCoM_des = configStateMachine.posCoM(currentState, :)';
    smoothingTime = configStateMachine.smoothingTime(currentState);
    useSmoothing = configStateMachine.useSmoothing(currentState);
    useForwardKinematicsForCoM = configStateMachine.useForwardKinematicsForCoM(currentState);
    
end
    
currentStateOut = currentState;

end
