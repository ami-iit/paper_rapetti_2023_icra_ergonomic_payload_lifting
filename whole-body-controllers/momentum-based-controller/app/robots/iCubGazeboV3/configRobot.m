% CONFIGROBOT initializes parameters specific of a particular robot
%             (e.g., icubGazeboSim)

%% --- Initialization ---
ConfigRobot.ROBOT_DOF         = 23;


% Robot configuration for WBToolbox
WBTConfigRobot           = WBToolbox.Configuration;
WBTConfigRobot.RobotName = 'icubSim';
WBTConfigRobot.UrdfFile  = 'model.urdf';
WBTConfigRobot.LocalName = 'WBT';

% Controlboards and joints list. Each joint is associated to the corresponding controlboard 
WBTConfigRobot.ControlBoardsNames     = {'torso','left_arm','right_arm','left_leg','right_leg'};
WBTConfigRobot.ControlledJoints       = [];
ConfigRobot.numOfJointsForEachControlboard = [];

ControlBoards                                        = struct();
ControlBoards.(WBTConfigRobot.ControlBoardsNames{1}) = {'torso_pitch','torso_roll','torso_yaw'};
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

% Load Saved Joints configurations
run('./jointPositions.m')


%% -- Gains --
% postural
ConfigRobot.Gains.GainPostural.Kp = diag([180    60    60,                       ...    % torso
                                          130    80    20     60,                ...    % left arm
                                          130    80    20     60,                ...    % right arm
                                          150    20    20    180   140    20,    ...    % left leg
                                          150    20    20    180   140    20]) * 2.5;   % right leg

ConfigRobot.Gains.GainPostural.Kd = 2 * ((ConfigRobot.Gains.GainPostural.Kp).^(1/2)) / 10;

% torque minimization weight
ConfigRobot.Gains.tauMinimization = ones(ConfigRobot.ROBOT_DOF , 1);

% momentum
ConfigRobot.Gains.GainMomentum.Ki = [0; 0; 0; 0; 0; 0]; % only angular part is implemented
ConfigRobot.Gains.GainMomentum.Kp = [50; 50; 100; 0.25; 0.25; 0.25];
ConfigRobot.Gains.GainMomentum.Kd = 2 * ((ConfigRobot.Gains.GainMomentum.Kp).^(1/2)) / 10;


%% -- Regularization --
ConfigRobot.Reg.LambdaInvDump = 1;
ConfigRobot.Reg.AInvTol = 1e-5;
ConfigRobot.Reg.Hessian = 1e-7*eye(24);

%% -- State Machine --
ConfigRobot.StateMachine.stateType = [1; 2; 2];
ConfigRobot.StateMachine.useSmoothing = [1; 1; 1];
ConfigRobot.StateMachine.smoothingTime = [3; 3; 5;];
ConfigRobot.StateMachine.useForwardKinematicsForCoM = ones(3, 1);
ConfigRobot.StateMachine.loop = true;
             
ConfigRobot.StateMachine.jointPos = [zeros(1,ConfigRobot.ROBOT_DOF);
                                     jointPos('Height35');
                                     jointPos('Height75')];
                                 
ConfigRobot.StateMachine.posCoM = zeros(3,3);
ConfigRobot.StateMachine.tEnd = [4; 8; inf]; 


%% -- Constraints for force optimization -- 

% Wrench equality constraints
ConfigRobot.wrench_Aeq_const = [zeros(12,12), eye(12,12)];
mass = 0.0;
ConfigRobot.wrench_beq_const = [0; 0; (mass/2) * -9.81; zeros(3,1); 0; 0; (mass/2) * -9.81; zeros(3,1);];

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
feet_size                    = [-0.12  0.12 ;    % xMin, xMax
                                -0.05 0.05 ];   % yMin, yMax  
                                                    
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
