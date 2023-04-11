figure()
cmap = get(gca,'colororder');

subplot(1,2,1)
hold on
subplot(1,2,2)
hold on

load('wholeOptiHighReg.mat')
subplot(1,2,1)
tau = out.robot1_tau_coupledDynamics.Data;
tauNorm = zeros(1,size(tau,2));
for i=1:size(tau,2)
    tauNorm = tauNorm + tau(:,i).^2;
end
tauNorm = sqrt(tauNorm);plot(out.robot1_tauDes.Time, tauNorm, 'Color', cmap(1, :))
subplot(1,2,1)
tau = out.robot2_tau_coupledDynamics.Data;
tauNorm = zeros(1,size(tau,2));
for i=1:size(tau,2)
    tauNorm = tauNorm + tau(:,i).^2;
end
tauNorm = sqrt(tauNorm);
plot(out.robot1_tauDes.Time, tauNorm, 'Color', cmap(2, :))
legend('Robot 1', 'Robot 2')
xlabel('TIme [s]')
ylabel('Torque Norm [Nm]')

load('wholeOptiHighReg2.mat')
subplot(1,2,1)
tau = out.robot1_tau_coupledDynamics.Data;
tauNorm = zeros(1,size(tau,2));
for i=1:size(tau,2)
    tauNorm = tauNorm + tau(:,i).^2;
end
tauNorm = sqrt(tauNorm);plot(out.robot1_tauDes.Time, tauNorm, 'Color', cmap(1, :))
subplot(1,2,1)
tau = out.robot2_tau_coupledDynamics.Data;
tauNorm = zeros(1,size(tau,2));
for i=1:size(tau,2)
    tauNorm = tauNorm + tau(:,i).^2;
end
tauNorm = sqrt(tauNorm);
plot(out.robot1_tauDes.Time, tauNorm, 'Color', cmap(2, :))
xlabel('TIme [s]')
ylabel('Torque Norm [Nm]')


load('wholeOptiHighReg3.mat')
subplot(1,2,1)
tau = out.robot1_tau_coupledDynamics.Data;
tauNorm = zeros(1,size(tau,2));
for i=1:size(tau,2)
    tauNorm = tauNorm + tau(:,i).^2;
end
tauNorm = sqrt(tauNorm);plot(out.robot1_tauDes.Time, tauNorm, 'Color', cmap(1, :))
subplot(1,2,1)
tau = out.robot2_tau_coupledDynamics.Data;
tauNorm = zeros(1,size(tau,2));
for i=1:size(tau,2)
    tauNorm = tauNorm + tau(:,i).^2;
end
tauNorm = sqrt(tauNorm);
plot(out.robot1_tauDes.Time, tauNorm, 'Color', cmap(2, :))
xlabel('TIme [s]')
ylabel('Torque Norm [Nm]')



load('wholeOptiLowReg.mat')
subplot(1,2,2)
tau = out.robot1_tau_coupledDynamics.Data;
tauNorm = zeros(1,size(tau,2));
for i=1:size(tau,2)
    tauNorm = tauNorm + tau(:,i).^2;
end
tauNorm = sqrt(tauNorm);
plot(out.robot1_tauDes.Time, tauNorm, 'Color', cmap(1, :))
subplot(1,2,2)
tau = out.robot2_tau_coupledDynamics.Data;
tauNorm = zeros(1,size(tau,2));
for i=1:size(tau,2)
    tauNorm = tauNorm + tau(:,i).^2;
end
tauNorm = sqrt(tauNorm);
plot(out.robot1_tauDes.Time, tauNorm, 'Color', cmap(2, :))
xlabel('TIme [s]')
ylabel('Torque Norm [Nm]')


load('wholeOptiLowReg2.mat')
subplot(1,2,2)
tau = out.robot1_tau_coupledDynamics.Data;
tauNorm = zeros(1,size(tau,2));
for i=1:size(tau,2)
    tauNorm = tauNorm + tau(:,i).^2;
end
tauNorm = sqrt(tauNorm);
plot(out.robot1_tauDes.Time, tauNorm, 'Color', cmap(1, :))


subplot(1,2,2)
tau = out.robot2_tau_coupledDynamics.Data;
tauNorm = zeros(1,size(tau,2));
for i=1:size(tau,2)
    tauNorm = tauNorm + tau(:,i).^2;
end
tauNorm = sqrt(tauNorm);
plot(out.robot1_tauDes.Time, tauNorm, 'Color', cmap(2, :))
xlabel('TIme [s]')
ylabel('Torque Norm [Nm]')


load('wholeOptiLowReg3.mat')
subplot(1,2,2)
tau = out.robot1_tau_coupledDynamics.Data;
tauNorm = zeros(1,size(tau,2));
for i=1:size(tau,2)
    tauNorm = tauNorm + tau(:,i).^2;
end
tauNorm = sqrt(tauNorm);
plot(out.robot1_tauDes.Time, tauNorm, 'Color', cmap(1, :))
subplot(1,2,2)
tau = out.robot2_tau_coupledDynamics.Data;
tauNorm = zeros(1,size(tau,2));
for i=1:size(tau,2)
    tauNorm = tauNorm + tau(:,i).^2;
end
tauNorm = sqrt(tauNorm);
plot(out.robot1_tauDes.Time, tauNorm, 'Color', cmap(2, :))
xlabel('TIme [s]')
ylabel('Torque Norm [Nm]')





