% plotSimSolution - Script to plot output data from Simulink
%
% Dual-Agent CPT Problem
% Copyright 2019
%
% Adapted from the Supersonic Aircraft Minimum Fuel Climb example for
% ICLOCS Version 2 (2018).
% The contribution of Yuanbo Nie, Omar Faqir, and Eric Kerrigan for their
% work on ICLOCS Version 2 (2018) is kindly acknowledged.
% Department of Electrical and Electronic Engineering,
% Imperial College London, UK
%--------------------------------------------------------
xx=linspace(solution.T(1,1),solution.T(end,1),1000);
    solution.tf=5;
    figure('Name','Closed Loop States')
    subplot(4,2,1)
    plot(X.time,X.data(:,1),'r.' )
    hold on
    plot(X.time,X.data(:,13),'g.' )
    hold on
    plot(X.time,X.data(:,7),'k.' )
    hold on
    plot(X.time,X.data(:,19),'k.' )
%     ylim([0 10])
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('X Position [m]')
    legend('X_1','X_2','X_{c1}','X_{c2}')
    grid on
    
    subplot(4,2,2)
    plot(X.time,X.data(:,2),'r.' )
    hold on
    plot(X.time,X.data(:,14),'g.' )
    hold on
    plot(X.time,X.data(:,8),'k.' )
    hold on
    plot(X.time,X.data(:,20),'k.' )
%     ylim([0 10])
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('X Velocity [m.s^{-1}]')
    legend('X_1','X_2','X_{c1}','X_{c2}')
    grid on
    
    subplot(4,2,3)
    plot(X.time,X.data(:,3),'r.' )
    hold on
    plot(X.time,X.data(:,15),'g.' )
    hold on
    plot(X.time,X.data(:,9),'k.' )
    hold on
    plot(X.time,X.data(:,21),'k.' )
    ylim([-1 1])
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('Y Position [m]')
    legend('Y_1','Y_2','Y_{c1}','Y_{c2}')
    grid on
    
    subplot(4,2,4)
    plot(X.time,X.data(:,4),'r.' )
    hold on
    plot(X.time,X.data(:,16),'g.' )
    hold on
    plot(X.time,X.data(:,10),'k.' )
    hold on
    plot(X.time,X.data(:,22),'k.' )
    ylim([-1 1])
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('Y Velocity [m.s^{-1}]')
    legend('Y_1','Y_2','Y_{c1}','Y_{c2}')
    grid on
    
    subplot(4,2,5)
    plot(X.time,X.data(:,5),'r.' )
    hold on
    plot(X.time,X.data(:,17),'g.' )
    hold on
    plot(X.time,X.data(:,11),'k.' )
    hold on
    plot(X.time,X.data(:,23),'k.' )
%     ylim([0 5])
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('Z Position [m]')
    legend('Z_1','Z_2','Z_{c1}','Z_{c2}')
    grid on
    
    subplot(4,2,6)
    plot(X.time,X.data(:,6),'r.' )
    hold on
    plot(X.time,X.data(:,18),'g.' )
    hold on
    plot(X.time,X.data(:,12),'k.' )
    hold on
    plot(X.time,X.data(:,24),'k.' )
%     ylim([0 5])
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('Z Velocity [m.s^{-1}]')
    legend('Z_1','Z_2','Z_{c1}','Z_{c2}')
    grid on
    
    subplot(4,2,7)
    plot(X.time,(X.data(:,1)-X.data(:,7))./(X.data(:,5)-X.data(:,11)),'rx' )
    hold on
    plot(X.time,(X.data(:,3)-X.data(:,9))./(X.data(:,5)-X.data(:,11)),'gx' )
    hold on
    plot(X.time,(X.data(:,13)-X.data(:,19))./(X.data(:,17)-X.data(:,23)),'ro' )
    hold on
    plot(X.time,(X.data(:,15)-X.data(:,21))./(X.data(:,17)-X.data(:,23)),'go' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('States')
    legend('tan(\alpha)_1','tan(\beta)_1','tan(\alpha)_2','tan(\beta)_2')
    grid on
    
    subplot(4,2,8)
    plot(X.time,U(:,4),'r.' )
    hold on
    plot(X.time,U(:,8),'g.' )
    hold on
    xlim([0 solution.tf])
    xlabel('Time [s]')
    grid on
    ylabel('Cable Tension [N]')
    legend('F_{1}','F_{2}')
        
    figure('Name','Closed Loop Inputs')
    subplot(3,1,1)
    plot(T(:,1),U(:,1),'r.' )
    hold on
    plot(T(:,1),U(:,5),'g.' )
    hold on
    xlim([0 solution.tf])
    xlabel('Time [s]')
    grid on
    ylabel('Control Input [N]')
    legend('u_{X1}','u_{X2}')
    
    subplot(3,1,2)
    plot(T(:,1),U(:,2),'r.' )
    hold on
    plot(T(:,1),U(:,6),'g.' )
    hold on
    xlim([0 solution.tf])
    ylim([-1 1])
    xlabel('Time [s]')
    grid on
    ylabel('Control Input [N]')
    legend('u_{Y1}','u_{Y2}')
    
    subplot(3,1,3)
    plot(T(:,1),U(:,3),'r.' )
    hold on
    plot(T(:,1),U(:,7),'g.' )
    hold on
    xlim([0 solution.tf])
    ylim([16 19])
    xlabel('Time [s]')
    grid on
    ylabel('Control Input [N]')
    legend('u_{Z1}','u_{Z2}')