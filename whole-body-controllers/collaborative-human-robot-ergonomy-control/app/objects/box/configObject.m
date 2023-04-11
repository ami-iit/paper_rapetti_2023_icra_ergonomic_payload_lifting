% Inertia parameters (from board.sdf)
% the board frame is assumed to be z-up and x-y along the axis of the board
% with x pointing from the handlings of robot 1 towards the handlings of
% robot 2, and the center in the center of the board

% Mass
m = 2.5;
ConfigObject.m = m;

% Dimension
size_x = 0.34;
size_y = 0.24;
size_z = 0.025;
ConfigObject.size = [size_x size_y size_z];

% compute mass matrix for unitary mass
ConfigObject.MassMatrix = rsl.computeBoxMassMatrix(m, size_x, size_y, size_z);

% Handlings position
ConfigObject.contacts = [-size_x/2  size_y/4 size_z;  % robot 1 l_hand 
                         -size_x/2 -size_y/4 size_z;  % robot 1 r_hand
                          size_x/2 -size_y/4 size_z;  % robot 2 l_hand              
                          size_x/2  size_y/4 size_z]; % robot 2 r_hand

%% -- Gains --
% momentum
ConfigObject.Gains.GainMomentum.Ki = [0; 0; 0; 0; 5 * 0; 0.0]; % only angular part is implemented
ConfigObject.Gains.GainMomentum.Kp = [ 50;  50;  100; 1.25; 2.5; 1.25];
ConfigObject.Gains.GainMomentum.Kd = 0 * ((ConfigRobot.Gains.GainMomentum.Kp).^(1/2)) / 10;

%% -- Constraints for force optimization -- 
ConfigObject.linksUseConstraintMatrix = [0, 0, 0, 0];