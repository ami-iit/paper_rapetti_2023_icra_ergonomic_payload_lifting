% Inertia parameters (from board.sdf)
% the board frame is assumed to be z-up and x-y along the axis of the board
% with x pointing from the handlings of robot 1 towards the handlings of
% robot 2, and the center in the center of the board

% Mass
m = 2;
ConfigObject.m = m;

% Dimension
size_x = 0.5;
size_y = 0.5;
size_z = 0.025;
ConfigObject.size = [size_x size_y size_z];

% compute mass matrix for unitary mass
ConfigObject.MassMatrix = rsl.computeBoxMassMatrix(m, size_x, size_y, size_z);

% Handlings position
ConfigObject.contacts = [-size_x/2  size_y/4 size_z;  % robot 1 l_hand 
                         -size_x/2 -size_y/4 size_z;  % robot 1 r_hand
                          size_x/2 -size_y/4 size_z;  % robot 2 l_hand              
                          size_x/2  size_y/4 size_z]; % robot 2 r_hand

% Jacobians (velocity transform)
J_l_hand_1 = [     eye(3), -wbc.skew(ConfigObject.contacts(1,:));
               zeros(3,3),                                eye(3)];
J_r_hand_1 = [     eye(3), -wbc.skew(ConfigObject.contacts(2,:));
               zeros(3,3),                                eye(3)];
J_l_hand_2 = [     eye(3), -wbc.skew(ConfigObject.contacts(3,:));
               zeros(3,3),                                eye(3)];
J_r_hand_2 = [     eye(3), -wbc.skew(ConfigObject.contacts(4,:));
               zeros(3,3),                                eye(3)];
           
ConfigObject.Jacobian = [J_l_hand_1; J_r_hand_1; J_l_hand_2; J_r_hand_2];