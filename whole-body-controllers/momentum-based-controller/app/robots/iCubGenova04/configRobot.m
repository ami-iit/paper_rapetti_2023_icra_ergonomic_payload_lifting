% CONFIGROBOT initializes parameters specific of a particular robot
%             (e.g., icubGazeboSim)

%% --- Initialization ---
ConfigRobot.ROBOT_DOF         = 23;


% Robot configuration for WBToolbox
WBTConfigRobot           = WBToolbox.Configuration;
WBTConfigRobot.RobotName = 'icub';
WBTConfigRobot.UrdfFile  = 'iCubGenova04/model.urdf';
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

% Wrench ports
ConfigRobot.Ports.WRENCH_LEFT_FOOT  = '';
ConfigRobot.Ports.WRENCH_RIGHT_FOOT = '';


%% -- Gains --
% postural
ConfigRobot.Gains.GainPostural.Kp = diag([300   130   130,                       ...    % torso
                                          200    50    50     60,                ...    % left arm
                                          200    50    50     60,                ...    % right arm
                                          300    80    20    300   200    20,    ...    % left leg
                                          300    80    20    300   200    20]) * 2.5;   % right leg


ConfigRobot.Gains.GainPostural.Kd = 2 * ((ConfigRobot.Gains.GainPostural.Kp).^(1/2)) / 10;

% torque minimization weight
ConfigRobot.Gains.tauMinimization = ones(ConfigRobot.ROBOT_DOF , 1);

% momentum
ConfigRobot.Gains.GainMomentum.Ki = [0; 0; 0; 0; 0; 0]; % only angular part is implemented
ConfigRobot.Gains.GainMomentum.Kp = [50; 50; 10; 0.25; 0.25; 0.25];
ConfigRobot.Gains.GainMomentum.Kd = 2 * ((ConfigRobot.Gains.GainMomentum.Kp).^(1/2)) / 10;


%% -- Regularization --
ConfigRobot.Reg.LambdaInvDump = 1;
ConfigRobot.Reg.AInvTol = 1e-5; 
ConfigRobot.Reg.Hessian = 1e-7;

%% -- State Machine --
ConfigRobot.StateMachine.stateType = [1; 2; 2];
ConfigRobot.StateMachine.useSmoothing = [1; 1; 1];
ConfigRobot.StateMachine.smoothingTime = [3; 3; 10];
ConfigRobot.StateMachine.useForwardKinematicsForCoM = ones(3, 1);
ConfigRobot.StateMachine.loop = true;

jointPosUp_75 = [-0.15776           0           0 ...
                 -0.57755     0.42632    -0.20018      1.1742 ...
                 -0.57755     0.42632    -0.20018      1.1742...
                  0.072213    0.087084   0.0088584   -0.087318   -0.042648   -0.087617 ...
                  0.072213    0.087084   0.0088584   -0.087318   -0.042648   -0.087617];
           
jointPosUp_55 = [-0.26179           0           0 ...
                 -0.46899     0.35057     0.10975     0.34907 ...
                 -0.46899     0.35057     0.10975     0.34907 ...
                  0.2548    0.087034   0.0048554    -0.20091    -0.12069   -0.087611 ...
                  0.2548    0.087034   0.0048554    -0.20091    -0.12069   -0.087611];

jointPosDown = [ 1.0584           0           0 ...
                -1.4835     0.35076   -0.017568     0.61366 ...
                -1.4835     0.35076   -0.017568     0.61366  ...
                 1.309    0.070688   -0.098529     -1.4875    -0.43633    -0.12091 ...
                 1.309    0.070688   -0.098529     -1.4875    -0.43633    -0.12091];

ConfigRobot.StateMachine.jointPos = [zeros(1,ConfigRobot.ROBOT_DOF);
                                     jointPosDown;
                                     jointPosUp_55];
                                 
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
feet_size                    = [-0.07  0.12 ;    % xMin, xMax
                                -0.045 0.05 ];   % yMin, yMax  
                                                    
% Compute contact constraints (friction cone, unilateral constraints)
[ConfigRobot.ConstraintsMatrix, ConfigRobot.bVectorConstraints] = wbc.computeRigidContactConstraints ...
    (forceFrictionCoefficient, numberOfPoints, torsionalFrictionCoefficient, feet_size, fZmin);

%% -- Safety --
ConfigRobot.Safety.torqueSaturation = 100;
ConfigRobot.Safety.EMERGENCY_STOP_WITH_JOINTS_LIMITS = true;
ConfigRobot.Safety.jointLimitTollerance = -1*pi/180;
ConfigRobot.Safety.EMERGENCY_STOP_WITH_ENCODER_SPIKES = true;
ConfigRobot.Safety.maxJointsPositionDelta = 15*pi/180;
ConfigRobot.Safety.EMERGENCY_STOP_WITH_JOINTS_TORQUE_LIMITS = true;
ConfigRobot.Safety.jointTorqueLimitTollerance = 0;