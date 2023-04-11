%% General
Config.ON_GAZEBO         = false;
Config.GRAV_ACC          = 9.81;

%% Visualization
Config.Scopes.TIME_SCOPE = true;

%% Initial configuration
ConfigRobot1.w_H_base = [  1   0 0   0;
                           0   1 0   0;
                           0   0 1   0;
                           0   0 0   1];
                       
ConfigRobot2.w_H_base = [ -1   0 0   1;
                           0  -1 0   0;
                           0   0 1   0;
                           0   0 0   1];
                       
                       
% Total system configuration
Config.fixedLinkVelcityComponents = [ones(12,1); ones(3,1); zeros(3,1); ones(3,1); zeros(3,1); ones(12,1); ones(3,1); zeros(3,1); ones(3,1); zeros(3,1); ]; % ones(48,1);
                                 
Config.Reg.LambdaInvDump = 1;
Config.Reg.AInvTol = 1e-2;

Config.Reg.DesWrenchReg = diag([1e-15 * ones(12,1); 100 * ones(12,1); 10 * ones(12,1); 10 * ones(12,1)]);
Config.Reg.Hessian = 1e-2 * diag([ones(12,1); ones(12,1); ones(12,1); ones(12,1)]);

% Config.Reg.Hessian(13,13) = 10;
% Config.Reg.Hessian(19,19) = 10;

% Config.Reg.Hessian(24+13,24+13) = 10;
% Config.Reg.Hessian(24+19,24+19) = 10;



Config.wrench_Aeq_const = zeros(14,48);
Config.wrench_Aeq_const(1:3,16:18) = eye(3);
Config.wrench_Aeq_const(4:6,22:24) = eye(3);
Config.wrench_Aeq_const(7:9,40:42) = eye(3);
Config.wrench_Aeq_const(10:12,46:48) = eye(3);
Config.wrench_beq_const = zeros(14,1);

% internal wrench on robot hand
Config.wrench_Aeq_const(13,20) = 1;
Config.wrench_beq_const(13) = 0; %-30;
Config.wrench_Aeq_const(14,14) = 1;
Config.wrench_beq_const(14) = 0; %30;


Config.Gains.tauMinimization = 1 * [1 * ones(ConfigRobot1.ROBOT_DOF , 1); 1 * ones(ConfigRobot2.ROBOT_DOF , 1)];

% CoM minimum jerk
Config.com_minJerk_settlingTime = 1;

% Dummy gains
% GainPosturalDummy.Kp = diag([150    40   120,                       ...    % torso
%                         150    20    20     80,                ...    % left arm
%                         150    20    20     80,                ...    % right arm
%                         150    20    20    180   140    20,    ...    % left leg
%                         150    20    20    180   140    20]) * 0.0;   % right leg
GainPosturalDummy.Kp = diag(zeros(ConfigRobot2.ROBOT_DOF,1)) * 0.0;
                    
GainPosturalDummy.Kd = 0.0 * ((ConfigRobot.Gains.GainPostural.Kp).^(1/2)) / 10;
GainMomentumDummy.Ki = [0; 0; 0; 0; 0; 0]; % only angular part is implemented
GainMomentumDummy.Kp = [0; 0; 0; 0; 0; 0];
GainMomentumDummy.Kd = 0 * ((ConfigRobot.Gains.GainMomentum.Kp).^(1/2)) / 10;