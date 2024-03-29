jointPos = containers.Map();

jointPosition.jointNames = {'torso_pitch', ...
                            'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow', ...
                            'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow', ...
                            'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
                            'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};
jointPosition.jointValues = [ 0.95974713 ...
                             -1.54821993  0.2617994   0.09725471  0.44001482 ...
                             -1.54821993  0.2617994   0.09725471  0.44001482...
                              1.39626341  0.04335246 -0.10446215 -1.70215038 -0.52359878 -0.11369138 ...
                              1.39626341  0.04335246 -0.10446215 -1.70215038 -0.52359878 -0.11369138];
jointPos('Height25') = reorderJointPositions(jointPosition.jointValues, jointPosition.jointNames, WBTConfigRobot.ControlledJoints);


jointPosition.jointNames = {'torso_pitch', ...
                            'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow', ...
                            'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow', ...
                            'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
                            'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};
jointPosition.jointValues = [ 0.19310622 ...
                             -1.01744298  0.26179939  0.14646808  0.35093382 ...
                             -1.01744298  0.26179939  0.14646808  0.35093382...
                             1.39626341  0.05833275 -0.07208675 -1.41697558 -0.52359878 -0.09323314 ...
                             1.39626341  0.05833275 -0.07208675 -1.41697558 -0.52359878 -0.09323314];
jointPos('Height35') = reorderJointPositions(jointPosition.jointValues, jointPosition.jointNames, WBTConfigRobot.ControlledJoints);


jointPosition.jointNames = {'torso_pitch', ...
                            'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow', ...
                            'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow', ...
                            'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
                            'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};
jointPosition.jointValues = [-0.34906586 ...
                             -0.60628326  0.26179939  0.5293953   0.36644869 ...
                             -0.60628326  0.26179939  0.5293953   0.36644869...
                             1.11890619  0.06743586 -0.0437774 -1.10079332 -0.52359878 -0.08061978 ...
                             1.11890619  0.06743586 -0.0437774 -1.10079332 -0.52359878 -0.08061978];
jointPos('Height45') = reorderJointPositions(jointPosition.jointValues, jointPosition.jointNames, WBTConfigRobot.ControlledJoints);


jointPosition.jointNames = {'torso_pitch', ...
                            'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow', ...
                            'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow', ...
                            'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
                            'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};
jointPosition.jointValues = [-0.34906583 ...
                              0.53038558  0.2617994   0.67103913  0.36268727 ...
                              0.53038558  0.2617994   0.67103913  0.36268727...
                              0.48566387  0.06922395 -0.01328875 -0.42913352 -0.24301585 -0.07043955 ...
                              0.48566387  0.06922395 -0.01328875 -0.42913352 -0.24301585 -0.07043955];
jointPos('Height55') = reorderJointPositions(jointPosition.jointValues, jointPosition.jointNames, WBTConfigRobot.ControlledJoints);


jointPosition.jointNames = {'torso_pitch', ...
                            'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow', ...
                            'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow', ...
                            'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
                            'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};
jointPosition.jointValues = [-0.141982566 ...
                              -0.547579422  0.261800516  0.264453749  0.766188978 ...
                              -0.547579422  0.261800516  0.264453749  0.766188978 ...
                               0.0239137575 0.0688156291 -0.00115093674 -0.0267026859 -0.0139463454 -0.0688276959 ...
                               0.0239137575 0.0688156291 -0.00115093674 -0.0267026859 -0.0139463454 -0.0688276959];
jointPos('Height65') = reorderJointPositions(jointPosition.jointValues, jointPosition.jointNames, WBTConfigRobot.ControlledJoints);


jointPosition.jointNames = {'torso_pitch', ...
                            'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow', ...
                            'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow', ...
                            'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
                            'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};
jointPosition.jointValues = [-0.15776 ...
                             -0.57755     0.42632    -0.20018      1.1742 ...
                             -0.57755     0.42632    -0.20018      1.1742 ...
                              0.072213    0.087084   0.0088584   -0.087318   -0.042648   -0.087617 ...
                              0.072213    0.087084   0.0088584   -0.087318   -0.042648   -0.087617];
jointPos('PosUp_75') = reorderJointPositions(jointPosition.jointValues, jointPosition.jointNames, WBTConfigRobot.ControlledJoints);


jointPosition.jointNames = {'torso_pitch', ...
                            'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow', ...
                            'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow', ...
                            'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
                            'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};
jointPosition.jointValues = [-0.26179 ...
                             -0.46899     0.35057     0.10975     0.34907 ...
                             -0.46899     0.35057     0.10975     0.34907 ...
                             0.2548    0.087034   0.0048554    -0.20091    -0.12069   -0.087611 ...
                             0.2548    0.087034   0.0048554    -0.20091    -0.12069   -0.087611];
jointPos('PosUp_55') = reorderJointPositions(jointPosition.jointValues, jointPosition.jointNames, WBTConfigRobot.ControlledJoints);


jointPosition.jointNames = {'torso_pitch', ...
                            'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow', ...
                            'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow', ...
                            'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
                            'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};
jointPosition.jointValues = [ 1.0584 ...
                             -1.4835     0.35076   -0.017568     0.61366 ...
                             -1.4835     0.35076   -0.017568     0.61366  ...
                             1.309    0.070688   -0.098529     -1.4875    -0.43633    -0.12091 ...
                             1.309    0.070688   -0.098529     -1.4875    -0.43633    -0.12091];
jointPos('PosDown') = reorderJointPositions(jointPosition.jointValues, jointPosition.jointNames, WBTConfigRobot.ControlledJoints);



function jointPositionsOrdered = reorderJointPositions(jointPositions, jointNames, jointNamesOrdered)
    nJointsOrdered = length(jointNamesOrdered);
    jointPositionsOrdered = zeros(1, nJointsOrdered);
    for n = 1:nJointsOrdered
        index = find(ismember(jointNames, jointNamesOrdered{n}));
        if not(isempty(index))
            jointPositionsOrdered(n) = jointPositions(index);
        end
    end
end