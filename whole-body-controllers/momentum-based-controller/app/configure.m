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
