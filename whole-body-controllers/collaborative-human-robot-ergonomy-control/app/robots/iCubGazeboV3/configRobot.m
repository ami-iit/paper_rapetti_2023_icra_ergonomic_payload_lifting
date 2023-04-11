% CONFIGROBOT initializes parameters specific of a particular robot
%             (e.g., icubGazeboSim)

%% --- Initialization ---
ConfigRobot.ROBOT_DOF         = 22;


% Robot configuration for WBToolbox
WBTConfigRobot           = WBToolbox.Configuration;
WBTConfigRobot.RobotName = 'icubSim';
WBTConfigRobot.UrdfFile  = 'iCubGazeboV3/model.urdf';
WBTConfigRobot.LocalName = 'WBT';

% Controlboards and joints list. Each joint is associated to the corresponding controlboard 
WBTConfigRobot.ControlBoardsNames     = {'torso','left_arm','right_arm','left_leg','right_leg'};
WBTConfigRobot.ControlledJoints       = [];
ConfigRobot.numOfJointsForEachControlboard = [];

ControlBoards                                        = struct();
ControlBoards.(WBTConfigRobot.ControlBoardsNames{1}) = {'torso_roll','torso_yaw'};
ControlBoards.(WBTConfigRobot.ControlBoardsNames{2}) = {'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow'}; % , 'l_wrist_prosup'};
ControlBoards.(WBTConfigRobot.ControlBoardsNames{3}) = {'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow'}; % , 'r_wrist_prosup'};
ControlBoards.(WBTConfigRobot.ControlBoardsNames{4}) = {'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll'};
ControlBoards.(WBTConfigRobot.ControlBoardsNames{5}) = {'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};

for n = 1:length(WBTConfigRobot.ControlBoardsNames)

    WBTConfigRobot.ControlledJoints       = [WBTConfigRobot.ControlledJoints, ...
                                             ControlBoards.(WBTConfigRobot.ControlBoardsNames{n})];
    ConfigRobot.numOfJointsForEachControlboard = [ConfigRobot.numOfJointsForEachControlboard; length(ControlBoards.(WBTConfigRobot.ControlBoardsNames{n}))];
end

% Frames list
ConfigRobot.Frames.BASE       = 'root_link'; 
ConfigRobot.Frames.LEFT_FOOT  = 'l_sole';
ConfigRobot.Frames.RIGHT_FOOT = 'r_sole';
ConfigRobot.Frames.LEFT_HAND  = 'l_hand';
ConfigRobot.Frames.RIGHT_HAND = 'r_hand';
ConfigRobot.Frames.CHEST      = 'chest';

% if TRUE, the controller will STOP if the joints hit the joints limits
% and/or if the (unsigned) difference between two consecutive joints
% encoders measurements is greater than a given threshold.
ConfigRobot.Flag.EMERGENCY_STOP_WITH_JOINTS_LIMITS  = true;
ConfigRobot.Flag.EMERGENCY_STOP_WITH_ENCODER_SPIKES = true;

% Wrench ports
ConfigRobot.Ports.WRENCH_LEFT_FOOT  = '';
ConfigRobot.Ports.WRENCH_RIGHT_FOOT = '';


%% -- Gains --
% postural
ConfigRobot.Gains.GainPostural.Kp = diag([40   120,                              ...    % torso
                                          150    20    20     80,                ...    % left arm
                                          150    20    20     80,                ...    % right arm
                                          150    20    20    180   140    20,    ...    % left leg
                                          150    20    20    180   140    20]) * 5;   % right leg

ConfigRobot.Gains.GainPostural.Kd = 2 * ((ConfigRobot.Gains.GainPostural.Kp).^(1/2)) / 10;

% momentum
ConfigRobot.Gains.GainMomentum.Ki = [0; 0; 0; 0; 0; 0]; % only angular part is implemented
ConfigRobot.Gains.GainMomentum.Kp = [50; 50; 100; 0.25; 0.25; 0.25];
ConfigRobot.Gains.GainMomentum.Kd = 2 * ((ConfigRobot.Gains.GainMomentum.Kp).^(1/2)) / 10;

ConfigRobot.Gains.tauMinimization = ones(ConfigRobot.ROBOT_DOF , 1);


%% -- Regularization --
ConfigRobot.Reg.LambdaInvDump = 10;
ConfigRobot.Reg.AInvTol = 1;
ConfigRobot.Reg.Hessian = 10;

%% -- State Machine --
ConfigRobot.StateMachine.stateType = [1; 2; 3];
ConfigRobot.StateMachine.useSmoothing = [1; 1; 1];
ConfigRobot.StateMachine.smoothingTime = [1; 1; 1.3833 * 2];
ConfigRobot.StateMachine.useForwardKinematicsForCoM = ones(3, 1);
ConfigRobot.StateMachine.loop = false;

% Load Saved Joints configurations
run('./jointPositions.m')

             
ConfigRobot.StateMachine.jointPos = [zeros(1,ConfigRobot.ROBOT_DOF);
                                     jointPos('Height35');
                                     jointPos('Height75')];

ConfigRobot.StateMachine.baseDelta = [zeros(1,6);
                                      zeros(1,6);
                                      [0, 0, 40, 0, 0, 0]];

                                 
ConfigRobot.StateMachine.posCoM = zeros(3,3);
ConfigRobot.StateMachine.tEnd = [4; 8; inf]; 


% parametrized state machine configuration
fixedLinkVelcityComponents = [ones(1,6), ones(1,6), ones(1,2), zeros(1,4), ones(1,2), zeros(1,4)]; 
ConfigRobot.StateMachine.fixedLinkVelcityComponents = [fixedLinkVelcityComponents;
                                                       fixedLinkVelcityComponents;
                                                       fixedLinkVelcityComponents]; 

controlledLinkVelcityComponents = [zeros(1,12), zeros(1,2), 1, zeros(1,3), zeros(1,2), 1, zeros(1,3)]; 
ConfigRobot.StateMachine.controlledLinkVelcityComponents = [controlledLinkVelcityComponents;
                                                            controlledLinkVelcityComponents;
                                                            controlledLinkVelcityComponents]; 
                                          
ConfigRobot.StateMachine.controlledLinksKp = [30; 30; 100];
ConfigRobot.StateMachine.DeltaPhiMax = [0.05; 0.05; 0.05];


%% -- Constraints for force optimization -- 

% Wrench equality constraints
ConfigRobot.wrench_Aeq_const = [zeros(6,12), [zeros(3,3) eye(3,3) zeros(3, 6);zeros(3, 6) zeros(3,3) eye(3,3) ]];
ConfigRobot.wrench_beq_const = zeros(6,1);

% ConfigRobot.wrench_Aeq_const = [zeros(12,12), eye(12,12)];
% ConfigRobot.wrench_beq_const = zeros(12,1);

% Fixed component of the link velocities
ConfigRobot.fixedLinkVelcityComponents = [ones(6,1);
                                          ones(6,1); 
                                          zeros(6,1);
                                          zeros(6,1)];

% List of link where to use the constraints for force optimization
ConfigRobot.linksUseConstraintMatrix = [1, 1, 0, 0];

% The friction cone is approximated by using linear interpolation of the circle. 
% So, numberOfPoints defines the number of points used to interpolate the circle 
% in each cicle's quadrant
numberOfPoints               = 4;  
forceFrictionCoefficient     = 1/3;    
torsionalFrictionCoefficient = 1/75;
fZmin                        = 10;

% physical size of the foot                             
feet_size                    = [-0.07  0.12 ;    % xMin, xMax
                                -0.045 0.05 ];   % yMin, yMax  
                                                    
% Compute contact constraints (friction cone, unilateral constraints)
[ConfigRobot.ConstraintsMatrix, ConfigRobot.bVectorConstraints] = wbc.computeRigidContactConstraints ...
    (forceFrictionCoefficient, numberOfPoints, torsionalFrictionCoefficient, feet_size, fZmin);

%% -- Safety --
ConfigRobot.Safety.torqueSaturation = 500;
ConfigRobot.Safety.EMERGENCY_STOP_WITH_JOINTS_LIMITS = false;
ConfigRobot.Safety.jointLimitTollerance = 0.5*pi/180;
ConfigRobot.Safety.EMERGENCY_STOP_WITH_ENCODER_SPIKES = true;
ConfigRobot.Safety.maxJointsPositionDelta = 15*pi/180;
ConfigRobot.Safety.EMERGENCY_STOP_WITH_JOINTS_TORQUE_LIMITS = true;
ConfigRobot.Safety.jointTorqueLimitTollerance = 0;
