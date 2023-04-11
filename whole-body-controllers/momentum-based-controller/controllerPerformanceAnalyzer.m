%% Load Data
% load('data/removed_friction_on_torso.mat')

% selectedJoints = {'l_hip_pitch',  'l_hip_roll', 'l_hip_yaw', 'l_knee', 'l_ankle_pitch', 'l_ankle_roll'};
% selectedJoints = {'torso_pitch',  'torso_roll', 'torso_yaw'};
% selectedJoints = {'l_shoulder_pitch',  'l_shoulder_roll', 'l_shoulder_yaw', 'l_elbow'};
selectedJoints = {'r_hip_pitch', 'r_knee', 'r_ankle_pitch', 'l_hip_pitch', 'l_knee', 'l_ankle_pitch'};

% selectedJoints = {'torso_pitch'};

n_data = min([size(out.stateDes.jointPos.Data, 1), ...
              size(out.stateMeas.jointPos.Data, 1), ...
              size(out.tauDes.Data, 1), ...
              size(out.tauMeas.Data, 1)]); 


%% Position Tracking
figure()

subplot(2,1,1)
hold on

cmap = get(gca,'colororder');

for n=1:length(selectedJoints)

    jointIdx = find(contains(WBTConfigRobot_1.ControlledJoints, selectedJoints{n}));

    color = cmap(mod(n,7), :);

    plot(out.stateDes.jointPos.Time(1:n_data), out.stateDes.jointPos.Data(1:n_data,jointIdx), 'Color', color);
    
    plot(out.stateMeas.jointPos.Time(1:n_data), out.stateMeas.jointPos.Data(1:n_data,jointIdx), '--', 'Color', color);
    
end
plot_aesthetic('Desired (-) vs Measured(- -)', 'Time [s]', 'Joint Pos [rad]', '')
% legend (selectedJoints, 'Interpreter', 'none');



subplot(2,1,2)
hold on

for n=1:length(selectedJoints)
    
    jointIdx = find(contains(WBTConfigRobot_1.ControlledJoints, selectedJoints{n}));
    
    error = out.stateDes.jointPos.Data(1:n_data,jointIdx) - out.stateMeas.jointPos.Data(1:n_data,jointIdx);
    plot(out.stateDes.jointPos.Time(1:n_data), error);
        
end
legend (selectedJoints, 'Interpreter', 'none');

plot_aesthetic('Error (Desired - Measured)', 'Time [s]', 'Joint Pos [rad]', '')



%% CoM Tracking
figure()

subplot(2,1,1)
hold on

cmap = get(gca,'colororder');

measPosCoMData = squeeze(out.stateMeas.pos_CoM.Data)';

desPosCoMData = squeeze(out.stateDes.pos_CoM.Data)';

for n=1:3
    color = cmap(mod(n,7), :);

    plot(out.stateDes.pos_CoM.Time(1:n_data), desPosCoMData(1:n_data,n), 'Color', color);

    plot(out.stateMeas.pos_CoM.Time(1:n_data),measPosCoMData(1:n_data,n), '--', 'Color', color);

end
    
plot_aesthetic('Desired (-) vs Measured(- -)', 'Time [s]', 'CoM Pos [m]', '')
% legend (selectedJoints, 'Interpreter', 'none');



subplot(2,1,2)
hold on

for n=1:3
    
    error = desPosCoMData(1:n_data,n) - measPosCoMData(1:n_data,n);
    plot(out.stateDes.jointPos.Time(1:n_data), error);
        
end
legend ({'x', 'y', 'z'}, 'Interpreter', 'none');

plot_aesthetic('Error (Desired - Measured)', 'Time [s]', 'CoM Pos [m]', '')




%% Torque Tracking
figure()

subplot(2,1,1)
hold on

cmap = get(gca,'colororder');

for n=1:length(selectedJoints)
    
    jointIdx = find(contains(WBTConfigRobot_1.ControlledJoints, selectedJoints{n}));
    
    color = cmap(mod(n,7), :);
    
    plot(out.tauDes.Time(1:n_data), out.tauDes.Data(1:n_data,jointIdx), 'Color', color);
    
    plot(out.tauMeas.Time(1:n_data), out.tauMeas.Data(1:n_data,jointIdx), '--', 'Color', color);
    
end
plot_aesthetic('Desired (-) vs Measured(- -)', 'Time [s]', 'Torque [Nm]', '')
% legend (selectedJoints, 'Interpreter', 'none');


subplot(2,1,2)
hold on
title ('Error (Desired - Measured)')

for n=1:length(selectedJoints)
    
    jointIdx = find(contains(WBTConfigRobot_1.ControlledJoints, selectedJoints{n}));
    
    error = out.tauDes.Data(1:n_data,jointIdx) - out.tauMeas.Data(1:n_data,jointIdx);
    plot(out.tauDes.Time(1:n_data), error);
        
end
legend (selectedJoints, 'Interpreter', 'none');

plot_aesthetic('Error (Desired - Measured)', 'Time [s]', 'Torque [Nm]', '')


%% Wrenches

figure()
subplot(4,2,1)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,1:3))
plot_aesthetic('Left Foot Force', 'Time [s]', 'Force [Nm]', '')
subplot(4,2,2)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,4:6))
plot_aesthetic('Left Foot Torque', 'Time [s]', 'Torque [Nm]', '')

subplot(4,2,3)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,7:9))
plot_aesthetic('Right Foot Force', 'Time [s]', 'Force [Nm]', '')

subplot(4,2,4)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,10:12))
plot_aesthetic('Right Foot Torque', 'Time [s]', 'Torque [Nm]', '')

subplot(4,2,5)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,13:15))
plot_aesthetic('Left Hand Force', 'Time [s]', 'Force [Nm]', '')

subplot(4,2,6)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,16:18))
plot_aesthetic('Left Hand Torque', 'Time [s]', 'Torque [Nm]', '')

subplot(4,2,7)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,19:21))
plot_aesthetic('Right Hand Force', 'Time [s]', 'Force [Nm]', '')

subplot(4,2,8)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,22:24))
plot_aesthetic('Right Hand TOruqe', 'Time [s]', 'Torque [Nm]', '')


