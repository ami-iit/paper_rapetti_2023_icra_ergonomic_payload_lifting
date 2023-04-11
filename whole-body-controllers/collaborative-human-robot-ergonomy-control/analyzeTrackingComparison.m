figure()
cmap = get(gca,'colororder');



%% Reference
posDes = out.objectStateDes.link_pose.Data(1:3,4,:);
for axis=1:3
    subplot(3,1,axis)
    hold on
    plot(out.objectStateDes.link_pose.Time, squeeze(posDes(axis,1,:)), '--', 'Color', cmap(1, :));
end

%% Measured
load('independent_1.mat')
posMeas = out.objectStateMeas.link_pose.Data(1:3,4,:);
for axis=1:3
    subplot(3,1,axis)
    hold on
    plot(out.objectStateDes.link_pose.Time, squeeze(posMeas(axis,1,:)), 'Color', cmap(2, :));
end
load('collaborative_1.mat')
posMeas = out.objectStateMeas.link_pose.Data(1:3,4,:);
for axis=1:3
    subplot(3,1,axis)
    hold on
    plot(out.objectStateDes.link_pose.Time, squeeze(posMeas(axis,1,:)), 'Color', cmap(3, :));
end
load('coll_ind_1.mat')
posMeas = out.objectStateMeas.link_pose.Data(1:3,4,:);
for axis=1:3
    subplot(3,1,axis)
    hold on
    plot(out.objectStateDes.link_pose.Time, squeeze(posMeas(axis,1,:)), 'Color', cmap(4, :));
end
load('trajPar_1.mat')
posMeas = out.objectStateMeas.link_pose.Data(1:3,4,:);
for axis=1:3
    subplot(3,1,axis)
    hold on
    plot(out.objectStateDes.link_pose.Time, squeeze(posMeas(axis,1,:)), 'Color', cmap(5, :));
end

legend({'Reference', 'Independent Controllers', 'Collaborative Controller', 'Collaborative+Independent', 'Trajectory Parametrization'}, 'AutoUpdate','off')



load('independent_2.mat')
posMeas = out.objectStateMeas.link_pose.Data(1:3,4,:);
for axis=1:3
    subplot(3,1,axis)
    hold on
    plot(out.objectStateDes.link_pose.Time, squeeze(posMeas(axis,1,:)), 'Color', cmap(2, :));
end
load('independent_3.mat')
posMeas = out.objectStateMeas.link_pose.Data(1:3,4,:);
for axis=1:3
    subplot(3,1,axis)
    hold on
    plot(out.objectStateDes.link_pose.Time, squeeze(posMeas(axis,1,:)), 'Color', cmap(2, :));
end


load('collaborative_2.mat')
posMeas = out.objectStateMeas.link_pose.Data(1:3,4,:);
for axis=1:3
    subplot(3,1,axis)
    hold on
    plot(out.objectStateDes.link_pose.Time, squeeze(posMeas(axis,1,:)), 'Color', cmap(3, :));
end
load('collaborative_3.mat')
posMeas = out.objectStateMeas.link_pose.Data(1:3,4,:);
for axis=1:3
    subplot(3,1,axis)
    hold on
    plot(out.objectStateDes.link_pose.Time, squeeze(posMeas(axis,1,:)), 'Color', cmap(3, :));
end


load('coll_ind_2.mat')
posMeas = out.objectStateMeas.link_pose.Data(1:3,4,:);
for axis=1:3
    subplot(3,1,axis)
    hold on
    plot(out.objectStateDes.link_pose.Time, squeeze(posMeas(axis,1,:)), 'Color', cmap(4, :));
end
load('coll_ind_3.mat')
posMeas = out.objectStateMeas.link_pose.Data(1:3,4,:);
for axis=1:3
    subplot(3,1,axis)
    hold on
    plot(out.objectStateDes.link_pose.Time, squeeze(posMeas(axis,1,:)), 'Color', cmap(4, :));
end


load('trajPar_2.mat')
posMeas = out.objectStateMeas.link_pose.Data(1:3,4,:);
for axis=1:3
    subplot(3,1,axis)
    hold on
    plot(out.objectStateDes.link_pose.Time, squeeze(posMeas(axis,1,:)), 'Color', cmap(5, :));
end
load('trajPar_3.mat')
posMeas = out.objectStateMeas.link_pose.Data(1:3,4,:);
for axis=1:3
    subplot(3,1,axis)
    hold on
    plot(out.objectStateDes.link_pose.Time, squeeze(posMeas(axis,1,:)), 'Color', cmap(5, :));
end

