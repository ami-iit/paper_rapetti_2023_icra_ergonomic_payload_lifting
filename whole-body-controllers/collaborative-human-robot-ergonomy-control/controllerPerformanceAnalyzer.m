%% Load Data
%load('data/liftingWithWeight.mat')

%% Close plots
% close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ROBOT 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

selectedJoints = {'l_shoulder_roll',  'r_shoulder_roll','l_shoulder_pitch', 'l_shoulder_yaw', 'l_elbow'};
% selectedJoints = {'r_hip_pitch',  'r_hip_roll', 'r_hip_yaw', 'r_knee', 'r_ankle_pitch', 'r_ankle_roll'};
% selectedJoints = {'torso_roll', 'torso_yaw'};
% selectedJoints = {'l_shoulder_pitch',  'l_shoulder_roll', 'l_shoulder_yaw', 'l_elbow'};

n_data = min([size(out.robot1_stateDes.jointPos.Data, 1), ...
              size(out.robot1_stateMeas.jointPos.Data, 1), ...
              size(out.robot1_tauDes.Data, 1), ...
              size(out.robot1_tauMeas.Data, 1)]) - 10; 


%% Position Tracking
figure()

subplot(2,1,1)
hold on

cmap = get(gca,'colororder');

for n=1:length(selectedJoints)
    
    jointIdx = find(contains(WBTConfigRobot_1.ControlledJoints, selectedJoints{n}));
    
    plot(out.robot1_stateDes.jointPos.Time(1:n_data), out.robot1_stateDes.jointPos.Data(1:n_data,jointIdx), 'Color', cmap(n, :));
    
    plot(out.robot1_stateMeas.jointPos.Time(1:n_data), out.robot1_stateMeas.jointPos.Data(1:n_data,jointIdx), '--', 'Color', cmap(n, :));
    
end
plot_aesthetic('Desired (-) vs Measured(--)', 'Time [s]', 'Joint Pos [rad]', '')


subplot(2,1,2)
hold on

for n=1:length(selectedJoints)
    
    jointIdx = find(contains(WBTConfigRobot_1.ControlledJoints, selectedJoints{n}));
    
    error = out.robot1_stateDes.jointPos.Data(1:n_data,jointIdx) - out.robot1_stateMeas.jointPos.Data(1:n_data,jointIdx);
    plot(out.robot1_stateDes.jointPos.Time(1:n_data), error);
        
end
legend (selectedJoints, 'Interpreter', 'none');
plot_aesthetic('Error (Desired - Measured)', 'Time [s]', 'Joint Pos [rad]', '')


%% Velocities
figure()

subplot(2,1,1)
hold on

cmap = get(gca,'colororder');

for n=1:length(selectedJoints)
    
    jointIdx = find(contains(WBTConfigRobot_1.ControlledJoints, selectedJoints{n}));
        
    plot(out.robot1_stateMeas.jointPos.Time(1:n_data), out.robot1_stateMeas.jointPos.Data(1:n_data,jointIdx), '--', 'Color', cmap(n, :));
    
end
plot_aesthetic('Joint Position', 'Time [s]', 'Joint Pos [rad]', '')


subplot(2,1,2)
hold on

for n=1:length(selectedJoints)
    
    jointIdx = find(contains(WBTConfigRobot_1.ControlledJoints, selectedJoints{n}));
    
    plot(out.robot1_stateMeas.nu.Time(1:n_data), out.robot1_stateMeas.nu.Data(1:n_data,jointIdx+6), '--', 'Color', cmap(n, :));
        
end
plot(out.robot1_stateMeas.nu.Time(1:n_data), out.robot1_stateMeas.nu.Data(1:n_data,1:6), '-k');

legendForVelocities = selectedJoints;
legendForVelocities{length(selectedJoints)+1} = 'Base velocity';
legend (legendForVelocities, 'Interpreter', 'none');
plot_aesthetic('Velocities', 'Time [s]', 'Velocities [m/s] or[rad]', '')


%% Torque Tracking
figure()

subplot(2,1,1)
hold on

cmap = get(gca,'colororder');

for n=1:length(selectedJoints)
    
    jointIdx = find(contains(WBTConfigRobot_1.ControlledJoints, selectedJoints{n}));
    
    plot(out.robot1_tauDes.Time(1:n_data), out.robot1_tauDes.Data(1:n_data,jointIdx), 'Color', cmap(n, :));
    
    plot(out.robot1_tauMeas.Time(1:n_data), out.robot1_tauMeas.Data(1:n_data,jointIdx), '--', 'Color', cmap(n, :));
    
end
plot_aesthetic('Desired (-) vs Measured(--)', 'Time [s]', 'Torque [Nm]', '')
% legend (selectedJoints, 'Interpreter', 'none');

subplot(2,1,2)
hold on

for n=1:length(selectedJoints)
    
    jointIdx = find(contains(WBTConfigRobot_1.ControlledJoints, selectedJoints{n}));
    
    error = out.robot1_tauDes.Data(1:n_data,jointIdx) - out.robot1_tauMeas.Data(1:n_data,jointIdx);
    plot(out.robot1_tauDes.Time(1:n_data), error);
        
end
legend (selectedJoints, 'Interpreter', 'none');
plot_aesthetic('Error (Desired - Measured)', 'Time [s]', 'Torque [Nm]', '')


% %% Wrenches
% 
% figure()
% subplot(4,2,1)
% plot(out.wrenchDes.Time(1:n_data), out.wrenchDes1.Data(1:n_data,1:3))
% plot_aesthetic('Left Foot Force (single)', 'Time [s]', 'Force [N]', '')
% subplot(4,2,2)
% plot(out.wrenchDes.Time(1:n_data), out.wrenchDes1.Data(1:n_data,4:6))
% plot_aesthetic('Left Foot Torque (single)', 'Time [s]', 'Torque [Nm]', '')
% subplot(4,2,3)
% plot(out.wrenchDes.Time(1:n_data), out.wrenchDes1.Data(1:n_data,7:9))
% plot_aesthetic('Right Foot Force (single)', 'Time [s]', 'Force [N]', '')
% subplot(4,2,4)
% plot(out.wrenchDes.Time(1:n_data), out.wrenchDes1.Data(1:n_data,10:12))
% plot_aesthetic('Right Foot Torque (single)', 'Time [s]', 'Torque [Nm]', '')
% subplot(4,2,5)
% plot(out.wrenchDes.Time(1:n_data), out.wrenchDes1.Data(1:n_data,13:15))
% plot_aesthetic('Left Hand Force (single)', 'Time [s]', 'Force [N]', '')
% subplot(4,2,6)
% plot(out.wrenchDes.Time(1:n_data), out.wrenchDes1.Data(1:n_data,16:18))
% plot_aesthetic('Left Hand Torque (single)', 'Time [s]', 'Torque [Nm]', '')
% subplot(4,2,7)
% plot(out.wrenchDes.Time(1:n_data), out.wrenchDes1.Data(1:n_data,19:21))
% plot_aesthetic('Right Hand Force (single)', 'Time [s]', 'Force [N]', '')
% subplot(4,2,8)
% plot(out.wrenchDes.Time(1:n_data), out.wrenchDes1.Data(1:n_data,22:24))
% plot_aesthetic('Right Hand Torque (single)', 'Time [s]', 'Torque [Nm]', '')
% 

figure()
subplot(4,2,1)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,1:3))
plot_aesthetic('Left Foot Force (whole)', 'Time [s]', 'Force [N]', '')
subplot(4,2,2)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,4:6))
plot_aesthetic('Left Foot Torque (whole)', 'Time [s]', 'Torque [Nm]', '')
subplot(4,2,3)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,7:9))
plot_aesthetic('Right Foot Force (whole)', 'Time [s]', 'Force [N]', '')
subplot(4,2,4)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,10:12))
plot_aesthetic('Right Foot Torque (whole)', 'Time [s]', 'Torque [Nm]', '')
subplot(4,2,5)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,13:15))
plot_aesthetic('Left Hand Force (whole)', 'Time [s]', 'Force [N]', '')
subplot(4,2,6)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,16:18))
plot_aesthetic('Left Hand Torque (whole)', 'Time [s]', 'Torque [Nm]', '')
subplot(4,2,7)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,19:21))
plot_aesthetic('Right Hand Force (whole)', 'Time [s]', 'Force [N]', '')
subplot(4,2,8)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,22:24))
plot_aesthetic('Right Hand Torque (whole)', 'Time [s]', 'Torque [Nm]', '')

figure()
subplot(4,2,1)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,1:3) - out.wrenchAnalytic.Data(1:n_data,1:3))
plot_aesthetic('Left Foot Force (whole, f0 from QP)', 'Time [s]', 'Force [N]', '')
subplot(4,2,2)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,4:6) - out.wrenchAnalytic.Data(1:n_data,4:6))
plot_aesthetic('Left Foot Torque (whole, f0 from QP)', 'Time [s]', 'Torque [Nm]', '')
subplot(4,2,3)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,7:9) - out.wrenchAnalytic.Data(1:n_data,7:9))
plot_aesthetic('Right Foot Force (whole, f0 from QP)', 'Time [s]', 'Force [N]', '')
subplot(4,2,4)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,10:12) - out.wrenchAnalytic.Data(1:n_data,10:12))
plot_aesthetic('Right Foot Torque (whole, f0 from QP)', 'Time [s]', 'Torque [Nm]', '')
subplot(4,2,5)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,13:15) - out.wrenchAnalytic.Data(1:n_data,13:15))
plot_aesthetic('Left Hand Force (whole, f0 from QP)', 'Time [s]', 'Force [N]', '')
subplot(4,2,6)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,16:18) - out.wrenchAnalytic.Data(1:n_data,16:18))
plot_aesthetic('Left Hand Torque (whole, f0 from QP)', 'Time [s]', 'Torque [Nm]', '')
subplot(4,2,7)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,19:21) - out.wrenchAnalytic.Data(1:n_data,19:21))
plot_aesthetic('Right Hand Force (whole, f0 from QP)', 'Time [s]', 'Force [N]', '')
subplot(4,2,8)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,22:24) - out.wrenchAnalytic.Data(1:n_data,22:24))
plot_aesthetic('Right Hand Torque (whole, f0 from QP)', 'Time [s]', 'Torque [Nm]', '')



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% OBJECT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Position
figure()

subplot(2,1,1)
hold on

cmap = get(gca,'colororder');

for n=1:3
        
    plot(out.objectStateDes.link_pose.Time(1:n_data),squeeze(out.objectStateDes.link_pose.Data(n,4,1:n_data)), 'Color', cmap(n, :));
    
    plot(out.objectStateMeas.link_pose.Time(1:n_data),squeeze(out.objectStateMeas.link_pose.Data(n,4,1:n_data)), '--', 'Color', cmap(n, :));
    
end
plot_aesthetic('Object Position Desired (-) vs Measured(--)', 'Time [s]', 'Pos [m]', '')
legend ({'x', 'y', 'x'}, 'Interpreter', 'none');


% subplot(2,1,2)
% hold on
% for n=1:3
%     
%     error = squeeze(out.objectStateDes.link_pose.Data(n,4,1:n_data)) - squeeze(out.objectStateMeas.link_pose.Data(n,4,1:n_data));
%     plot(out.objectStateMeas.link_pose.Time(1:n_data), error);
%         
% end
% legend ({'x', 'y', 'x'}, 'Interpreter', 'none');
% plot_aesthetic('Object Position Error (Desired - Measured)', 'Time [s]', 'Pos [m]', '')



% % Orientation
% 
% figure()
% 
% % subplot(2,1,1)
% hold on
% cmap = get(gca,'colororder');

% compute roll-pitch-yaw
subplot(2,1,2)
hold on

objectRollPitchYawDes  = [];
objectRollPitchYawMeas = [];
for n=1:n_data
    
    objectRollPitchYawDes  = [objectRollPitchYawDes, wbc.rollPitchYawFromRotation(out.objectStateDes.link_pose.Data(1:3,1:3,n))];
    objectRollPitchYawMeas = [objectRollPitchYawMeas, wbc.rollPitchYawFromRotation(out.objectStateMeas.link_pose.Data(1:3,1:3,n))];
end

for n=1:3
        
    plot(out.objectStateDes.link_pose.Time(1:n_data),rad2deg(objectRollPitchYawDes(n,:)), 'Color', cmap(n, :));
    plot(out.objectStateMeas.link_pose.Time(1:n_data),rad2deg(objectRollPitchYawMeas(n,:)), '--', 'Color', cmap(n, :));
    
end
legend ({'roll', 'pitch', 'yaw'}, 'Interpreter', 'none');
plot_aesthetic('Object Orientation Desired (-) vs Measured(--)', 'Time [s]', 'Angle [deg]', '')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ROBOT 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

selectedJointsHuman = {'jLeftKnee_roty', ...
                       'jLeftAnkle_roty', ...
                       'jLeftHip_roty', ...
                       'jL5S1_roty', ...
                      };

%% Velocities
figure()

subplot(2,1,1)
hold on

cmap = get(gca,'colororder');

for n=1:length(selectedJointsHuman)
    
    jointIdx = find(contains(WBTConfigRobot_2.ControlledJoints, selectedJointsHuman{n}));
        
    plot(out.robot1_stateMeas.jointPos.Time(1:n_data), out.robot2_stateMeas.jointPos.Data(1:n_data,jointIdx), '--', 'Color', cmap(n, :));
    
end
plot_aesthetic('Human Joint Position', 'Time [s]', 'Joint Pos [rad]', '')

% figure()
subplot(2,1,2)
hold on

for n=1:length(selectedJointsHuman)
    
    jointIdx = find(contains(WBTConfigRobot_2.ControlledJoints, selectedJointsHuman{n}));
    
    plot(out.robot1_stateMeas.nu.Time(1:n_data), out.robot2_stateMeas.nu.Data(1:n_data,jointIdx+6), '--', 'Color', cmap(n, :));
        
end
% plot(out.robot1_stateMeas.nu.Time(1:n_data), out.robot2_stateMeas.nu.Data(1:n_data,1:6), '-');

legendForVelocities = selectedJointsHuman;
legendForVelocities{length(selectedJointsHuman)+1} = 'Base velocity';
% legendForVelocities = {'dx','dy','dz','wx','wy','wz' };
legend (legendForVelocities, 'Interpreter', 'none');
plot_aesthetic('Human Joint Velocities', 'Time [s]', 'Velocities [rad/s]', '')
% plot_aesthetic('Human Base Velocities', 'Time [s]', 'Velocities [m/s] or[rad/s]', '')

figure()
hold on

plot(out.robot1_stateMeas.nu.Time(1:n_data), out.robot2_stateMeas.nu.Data(1:n_data,1:6), '-');

legendForVelocities = {'dx','dy','dz','wx','wy','wz' };
legend (legendForVelocities, 'Interpreter', 'none');
plot_aesthetic('Human Base Velocities', 'Time [s]', 'Velocities [m/s] or[rad/s]', '')


figure()
subplot(4,2,1)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,25:27) - out.wrenchAnalytic.Data(1:n_data,25:27))
plot_aesthetic('Human Left Foot Force (whole, f0 from QP)', 'Time [s]', 'Force [N]', '')
subplot(4,2,2)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,28:30) - out.wrenchAnalytic.Data(1:n_data,28:30))
plot_aesthetic('Human Left Foot Torque (whole, f0 from QP)', 'Time [s]', 'Torque [Nm]', '')
subplot(4,2,3)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,31:33) - out.wrenchAnalytic.Data(1:n_data,31:33))
plot_aesthetic('Human Right Foot Force (whole, f0 from QP)', 'Time [s]', 'Force [N]', '')
subplot(4,2,4)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,34:36) - out.wrenchAnalytic.Data(1:n_data,34:36))
plot_aesthetic('Human Right Foot Torque (whole, f0 from QP)', 'Time [s]', 'Torque [Nm]', '')
subplot(4,2,5)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,37:39) - out.wrenchAnalytic.Data(1:n_data,37:39))
plot_aesthetic('Human Left Hand Force (whole, f0 from QP)', 'Time [s]', 'Force [N]', '')
subplot(4,2,6)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,40:42) - out.wrenchAnalytic.Data(1:n_data,40:42))
plot_aesthetic('Human Left Hand Torque (whole, f0 from QP)', 'Time [s]', 'Torque [Nm]', '')
subplot(4,2,7)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,43:45) - out.wrenchAnalytic.Data(1:n_data,43:45))
plot_aesthetic('Human Right Hand Force (whole, f0 from QP)', 'Time [s]', 'Force [N]', '')
subplot(4,2,8)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,46:48) - out.wrenchAnalytic.Data(1:n_data,46:48))
plot_aesthetic('Human Right Hand Torque (whole, f0 from QP)', 'Time [s]', 'Torque [Nm]', '')

figure()
subplot(4,2,1)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,25:27))
plot_aesthetic('Human Left Foot Force (whole)', 'Time [s]', 'Force [N]', '')
subplot(4,2,2)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,28:30))
plot_aesthetic('Human Left Foot Torque (whole)', 'Time [s]', 'Torque [Nm]', '')
subplot(4,2,3)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,31:33))
plot_aesthetic('Human Right Foot Force (whole)', 'Time [s]', 'Force [N]', '')
subplot(4,2,4)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,34:36))
plot_aesthetic('Human Right Foot Torque (whole)', 'Time [s]', 'Torque [Nm]', '')
subplot(4,2,5)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,37:39))
plot_aesthetic('Human Left Hand Force (whole)', 'Time [s]', 'Force [N]', '')
subplot(4,2,6)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,40:42))
plot_aesthetic('Human Left Hand Torque (whole)', 'Time [s]', 'Torque [Nm]', '')
subplot(4,2,7)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,43:45))
plot_aesthetic('Human Right Hand Force (whole)', 'Time [s]', 'Force [N]', '')
subplot(4,2,8)
plot(out.wrenchDes.Time(1:n_data), out.wrenchDes.Data(1:n_data,46:48))
plot_aesthetic('Human Right Hand Torque (whole)', 'Time [s]', 'Torque [Nm]', '')



%% CoM Desired

% Object
figure()

subplot(3,1,1)
hold on

for n=1:3
        
    plot(out.objectStateDes.link_pose.Time(1:n_data),squeeze(out.object_trajCoMDes.Data(3,1,1:n_data)), 'Color', cmap(n, :));
        
end
plot_aesthetic('Object Position Desired', 'Time [s]', 'Pos [m]', '')
legend ({'x', 'y', 'z'}, 'Interpreter', 'none');

subplot(3,1,2)
hold on

for n=1:3
        
    plot(out.objectStateDes.link_pose.Time(1:n_data),squeeze(out.object_trajCoMDes.Data(3,2,1:n_data)), 'Color', cmap(n, :));
        
end
plot_aesthetic('Object Velocity Desired', 'Time [s]', 'Vel [m/s]', '')
legend ({'x', 'y', 'z'}, 'Interpreter', 'none');

subplot(3,1,3)
hold on

for n=1:3
        
    plot(out.objectStateDes.link_pose.Time(1:n_data),squeeze(out.object_trajCoMDes.Data(3,3,1:n_data)), 'Color', cmap(n, :));
        
end
plot_aesthetic('Object Acceleration Desired', 'Time [s]', 'Acc [m/s^2]', '')
legend ({'x', 'y', 'z'}, 'Interpreter', 'none');

% Robot
figure()

subplot(3,1,1)
hold on

for n=1:3
        
    plot(out.objectStateDes.link_pose.Time(1:n_data),squeeze(out.robot1_trajCoMDes.Data(3,1,1:n_data)), 'Color', cmap(n, :));
        
end
plot_aesthetic('Robot Position Desired', 'Time [s]', 'Pos [m]', '')
legend ({'x', 'y', 'z'}, 'Interpreter', 'none');

subplot(3,1,2)
hold on

for n=1:3
        
    plot(out.objectStateDes.link_pose.Time(1:n_data),squeeze(out.robot1_trajCoMDes.Data(3,2,1:n_data)), 'Color', cmap(n, :));
        
end
plot_aesthetic('Robot Velocity Desired', 'Time [s]', 'Vel [m/s]', '')
legend ({'x', 'y', 'z'}, 'Interpreter', 'none');

subplot(3,1,3)
hold on

for n=1:3
        
    plot(out.objectStateDes.link_pose.Time(1:n_data),squeeze(out.robot1_trajCoMDes.Data(3,3,1:n_data)), 'Color', cmap(n, :));
        
end
plot_aesthetic('Robot Acceleration Desired', 'Time [s]', 'Acc [m/s^2]', '')
legend ({'x', 'y', 'z'}, 'Interpreter', 'none');

% Human
figure()

subplot(3,1,1)
hold on

plot(out.objectStateDes.link_pose.Time(1:n_data),squeeze(out.robot2_trajCoMDes.Data(1:3,1,1:n_data)));

plot_aesthetic('Human Position Desired', 'Time [s]', 'Pos [m]', '')
legend ({'x', 'y', 'z'}, 'Interpreter', 'none');

subplot(3,1,2)
hold on

plot(out.objectStateDes.link_pose.Time(1:n_data),squeeze(out.robot2_trajCoMDes.Data(1:3,2,1:n_data)));

plot_aesthetic('Human Velocity Desired', 'Time [s]', 'Vel [m/s]', '')
legend ({'x', 'y', 'z'}, 'Interpreter', 'none');

subplot(3,1,3)
hold on

plot(out.objectStateDes.link_pose.Time(1:n_data),squeeze(out.robot2_trajCoMDes.Data(1:3,3,1:n_data)));
        
plot_aesthetic('Human Acceleration Desired', 'Time [s]', 'Acc [m/s^2]', '')
legend ({'x', 'y', 'z'}, 'Interpreter', 'none');