% plotSimSolution - script to plot output data from Simulink
%
% Single Agent Relocation Problem
% Copyright 2019
%
% Adapted from the Supersonic Aircraft Minimum Fuel Climb example for
% ICLOCS Version 2 (2018).
% The contribution of Yuanbo Nie, Omar Faqir, and Eric Kerrigan for their
% work on ICLOCS Version 2 (2018) is kindly acknowledged.
% Department of Electrical and Electronic Engineering,
% Imperial College London, UK
%--------------------------------------------------------
    disp(toc)
    xx=linspace(solution.T(1,1),solution.T(end,1),1000);
    solution.tf=5;
    figure('Name','Closed Loop States')
    subplot(3,2,1)
    plot(X.time,X.data(:,1),'r.' )    
%     ylim([0 10])
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('X Position [m]')
%     legend('X_1')
    grid on
    
    subplot(3,2,2)
    plot(X.time,X.data(:,2),'r.' )
%     ylim([0 10])
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('X Velocity [ms^{-1}]')
%     legend('X_1')
    grid on
    
    subplot(3,2,3)
    plot(X.time,X.data(:,3),'g.' )
    ylim([-1 1])
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('Y Position [m]')
%     legend('Y_1')
    grid on
    
    subplot(3,2,4)
    plot(X.time,X.data(:,4),'g.' )
    ylim([-1 1])
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('Y Velocity [ms^{-1}]')
%     legend('Y_1')
    grid on
    
    subplot(3,2,5)
    plot(X.time,X.data(:,5),'b.' )
%     ylim([0 5])
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('Z Position [m]')
%     legend('Z_1')
    grid on
    
    subplot(3,2,6)
    plot(X.time,X.data(:,6),'b.' )
%     ylim([0 5])
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('Z Velocity [ms^{-1}]')
%     legend('Z_1')
    grid on
        
    figure('Name','Closed Loop Inputs')
    subplot(3,1,1)
    plot(T(:,1),U(:,1),'r.' )
    hold on
    xlim([0 solution.tf])
    xlabel('Time [s]')
    grid on
    ylabel('u_{X1} [N]')
%     legend('u_{X1}')
    
    subplot(3,1,2)
    plot(T(:,1),U(:,2),'g.' )
    hold on
    xlim([0 solution.tf])
%     ylim([-1 1])
    xlabel('Time [s]')
    grid on
    ylabel('u_{Y1} [N]')
%     legend('u_{Y1}')
    
    subplot(3,1,3)
    plot(T(:,1),U(:,3),'b.' )
    hold on
    xlim([0 solution.tf])
%     ylim([14 15])
    xlabel('Time [s]')
    grid on
    ylabel('u_{Z1} [N]')
%     legend('u_{Z1}')