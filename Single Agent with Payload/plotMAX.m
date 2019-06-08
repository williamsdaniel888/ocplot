% plotMAX - Plot output data from Simulink
%
% Single Agent with Payload Relocation Problem
% Copyright 2019
%
% Adapted from the Supersonic Aircraft Minimum Fuel Climb example for
% ICLOCS Version 2 (2018).
% The contribution of Yuanbo Nie, Omar Faqir, and Eric Kerrigan for their
% work on ICLOCS Version 2 (2018) is kindly acknowledged.
% Department of Electrical and Electronic Engineering,
% Imperial College London, UK
%--------------------------------------------------------
% close all    
xx=linspace(0,5,1000);
% comp_time = toc;
% xx=linspace(solution.T(1,1),solution.T(end,1),1000);
solution.tf = 5;
figure('Name','Closed Loop States')
subplot(4,2,1)
plot(X.time,X.data(:,1),'r.' )
hold on
plot(X.time,X.data(:,7),'g.' )
xlim([0 solution.tf])
xlabel('Time [s]')
ylabel('X Position [m]')
legend('X_1','X_{c1}')
grid on
subplot(4,2,2)
plot(X.time,X.data(:,2),'m.' )
hold on
plot(X.time,X.data(:,8),'b.' )
xlim([0 solution.tf])
xlabel('Time [s]')
ylabel('X Velocity [m.s^{-1}]')
legend('X_1','X_{c1}')
grid on

subplot(4,2,3)
plot(X.time,X.data(:,3),'r.' )
hold on
plot(X.time,X.data(:,9),'g.' )
xlim([0 solution.tf])
ylim([-1 1])
xlabel('Time [s]')
ylabel('Y Position [m]')
legend('Y_1','Y_{c1}')
grid on
subplot(4,2,4)
plot(X.time,X.data(:,4),'m.' )
hold on
plot(X.time,X.data(:,10),'b.' )
xlim([0 solution.tf])
ylim([-1 1])
xlabel('Time [s]')
ylabel('Y Velocity [m.s^{-1}]')
legend('Y_1','Y_{c1}')
grid on

subplot(4,2,5)
plot(X.time,X.data(:,5),'r.' )
hold on
plot(X.time,X.data(:,11),'g.' )
xlim([0 solution.tf])
xlabel('Time [s]')
ylabel('Z Position [m]')
legend('Z_1','Z_{c1}')
grid on

subplot(4,2,6)
plot(X.time,X.data(:,6),'m.' )
hold on
plot(X.time,X.data(:,12),'b.' )
xlim([0 solution.tf])
ylim([-1 1])
xlabel('Time [s]')
ylabel('Z Velocity [m.s^{-1}]')
legend('Z_1','Z_{c1}')
grid on

subplot(4,2,7)
plot(X.time,(X.data(:,1)-X.data(:,7))./(X.data(:,5)-X.data(:,11)),'r.' )
hold on
plot(X.time,(X.data(:,3)-X.data(:,9))./(X.data(:,5)-X.data(:,11)),'g.' )
xlim([0 solution.tf])
xlabel('Time [s]')
ylabel('States')
legend('tan(\alpha)','tan(\beta)')
grid on

subplot(4,2,8)
plot(X.time,U(:,4),'b.' )
xlim([0 solution.tf])
xlabel('Time [s]')
ylabel('Cable Tension [N]')
grid on

figure('Name','Closed Loop Control Inputs')
subplot(3,1,1)
plot(X.time,U(:,1),'r.' )
xlim([0 solution.tf])
xlabel('Time [s]')
ylabel('U_{X1} [N]')
grid on
subplot(3,1,2)
plot(X.time,U(:,2),'g.' )
xlim([0 solution.tf])
ylim([-1 1])
xlabel('Time [s]')
ylabel('U_{Y1} [N]')
grid on
subplot(3,1,3)
plot(X.time,U(:,3),'b.' )
xlim([0 solution.tf])
% ylim([26 28])
xlabel('Time [s]')
ylabel('U_{Z1} [N]')
grid on
% cl_data={X.time,U(:,1),U(:,2),U(:,3)};
% save('matlab.mat','cl_data','-append')
% load('matlab.mat')
% figure('Name','uX')
% plot(ol_data{1},ol_data{2},'x')
% hold on
% plot(cl_data{1},cl_data{2},'+')
% grid on
% figure('Name','uY')
% plot(ol_data{1},ol_data{3},'x')
% hold on
% plot(cl_data{1},cl_data{3},'+')
% grid on
% figure('Name','uZ')
% plot(ol_data{1},ol_data{4},'x')
% hold on
% plot(cl_data{1},cl_data{4},'+')
% grid on