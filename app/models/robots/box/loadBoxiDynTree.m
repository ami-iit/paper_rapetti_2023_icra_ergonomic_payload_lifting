% Secondly we load the structure of the sensors list
% from URDF


kinDynModel = iDynTreeWrappers.loadReducedModel([], 'base_link', './','box.urdf', false);

basePose = [ -1   0 0 1.5;
              0  -1 0   0;
              0   0 1   0;
              0   0 0   1];
iDynTreeWrappers.setRobotState(kinDynModel, basePose, [], zeros(6,1), [], [0; 0; -9.81]);

iDynTreeWrappers.getWorldTransform(kinDynModel,'base_link')
iDynTreeWrappers.getWorldTransform(kinDynModel,'side1_left_dummy_link')
iDynTreeWrappers.getWorldTransform(kinDynModel,'side1_right_dummy_link')
iDynTreeWrappers.getWorldTransform(kinDynModel,'side2_left_dummy_link')
iDynTreeWrappers.getWorldTransform(kinDynModel,'side2_right_dummy_link')
