xx=linspace(solution.T(1,1),solution.T(end,1),1000);

figure
subplot(3,1,1)
plot(solution.T(:,1),speval(solution.Xp,1,solution.T(:,1)),'ro' )
hold on
plot(solution.T(:,1),speval(solution.Xp,2,solution.T(:,1)),'md' )
plot(xx,speval(solution.Xp,1,xx),'r-' )
plot(xx,speval(solution.Xp,2,xx),'m-' )
ylim([0 35])
xlim([0 solution.tf])
xlabel('Time [s]')
ylabel('States')
legend('X Position [m]','X Velocity [m/s]')
grid on
subplot(3,1,2)
plot(solution.T(:,1),speval(solution.Xp,3,solution.T(:,1)),'go' )
hold on
plot(solution.T(:,1),speval(solution.Xp,4,solution.T(:,1)),'yd' )
plot(xx,speval(solution.Xp,3,xx),'g-' )
plot(xx,speval(solution.Xp,4,xx),'y-' )
ylim([0 35])
xlim([0 solution.tf])
xlabel('Time [s]')
ylabel('States')
legend('Y Position [m]','Y Velocity [m/s]')
grid on
subplot(3,1,3)
plot(solution.T(:,1),speval(solution.Xp,5,solution.T(:,1)),'bo' )
hold on
plot(solution.T(:,1),speval(solution.Xp,6,solution.T(:,1)),'cd' )
plot(xx,speval(solution.Xp,5,xx),'b-' )
plot(xx,speval(solution.Xp,6,xx),'c-' )
ylim([0 35])
xlim([0 solution.tf])
xlabel('Time [s]')
ylabel('States')
legend('Z Position [m]','Z Velocity [m/s]')
grid on

figure
subplot(3,1,1)
plot(solution.T(:,1),speval(solution.Up,1,solution.T),'k-o' )
hold on
plot([solution.T(1,1); solution.tf],[problem.inputs.ul(1), problem.inputs.ul(1)],'r-' )
plot([solution.T(1,1); solution.tf],[problem.inputs.uu(1), problem.inputs.uu(1)],'r-' )
xlim([0 solution.tf])
xlabel('Time [s]')
grid on
ylabel('Control Input')
legend('u_X [N]')
subplot(3,1,2)
plot(solution.T(:,1),speval(solution.Up,2,solution.T),'k-o' )
hold on
plot([solution.T(1,1); solution.tf],[problem.inputs.ul(2), problem.inputs.ul(2)],'r-' )
plot([solution.T(1,1); solution.tf],[problem.inputs.uu(2), problem.inputs.uu(2)],'r-' )
xlim([0 solution.tf])
xlabel('Time [s]')
grid on
ylabel('Control Input')
legend('u_Y [N]')
subplot(3,1,3)
plot(solution.T(:,1),speval(solution.Up,3,solution.T),'k-o' )
hold on
plot([solution.T(1,1); solution.tf],[problem.inputs.ul(3), problem.inputs.ul(3)],'r-' )
plot([solution.T(1,1); solution.tf],[problem.inputs.uu(3), problem.inputs.uu(3)],'r-' )
xlim([0 solution.tf])
xlabel('Time [s]')
grid on
ylabel('Control Input')
legend('u_Z [N]')
