%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
%  * @author: Lorenzo Rapetti
%  * Permission is granted to copy, distribute, and/or modify this program
%  * under the terms of the GNU General Public License, version 2 or any
%  * later version published by the Free Software Foundation.
%  *
%  * A copy of the license can be found at
%  * http://www.robotcub.org/icub/license/gpl.txt
%  *
%  * This program is distributed in the hope that it will be useful, but
%  * WITHOUT ANY WARRANTY; without even the implied warranty of
%  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  * Public License for more details
%  */
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc

% Add path to local source code
addpath('./src/')
% Add path to Multi-Body-Library
addpath('../matlab-library/multi-body-library/')
% Add path to Robot-State-Library
addpath('../matlab-library/robot-state-library/')

%% SIMULATION INFO

% Simulation time in seconds. For long simulations, put an high number 
% (not inf) for allowing automatic code generation
Config.SIMULATION_TIME = inf;

% Controller period [s]
Config.tStep           = 0.01;

%% PRELIMINARY CONFIGURATION

Robot1.MODEL                  = 'iCubGazeboV2_5';
Robot1.NAME                   = 'icubSim';

%% CONFIGURE ROBOT 1
% Run robot-specific configuration parameters
run(strcat('app/robots/',Robot1.MODEL,'/configRobot.m')); 

% save configuration for Robot 1
ConfigRobot1               = ConfigRobot;
WBTConfigRobot_1           = WBTConfigRobot.copy;
WBTConfigRobot_1.RobotName = Robot1.NAME;


%% GENERAL CONFIGURATION

% Load general configuration
run(strcat('app/configure.m')); 

%% Load Data
% load('data/dataset.mat')


