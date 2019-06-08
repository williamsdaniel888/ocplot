% main_OL - Script for open-loop simulations with ICLOCS2
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

clc;
clear all;
close all;
format compact;
addpath(genpath('..\..\ICLOCS\src'))
addpath(genpath('..\..\Ipopt'))
global sol;  
sol=[];                             % Initialize solution structure

options= settings_hscubconst(10);                  % Get options and solver settings 
[problem,guess]=MAGProblem;    % Fetch the problem definition

[infoNLP,data,options]=transcribeOCP(problem,guess,options); % Format for NLP solver
[solution,status,data] = solveNLP(infoNLP,data);      % Solve the NLP
[solution]=output(problem,solution,options,data,4);         % Output solutions

xx=linspace(solution.T(1,1),solution.tf,1000);

if (strcmp(options.transcription,'globalLGR')) || (strcmp(options.transcription,'hpLGR'))

    figure('Name','Open Loop States')
    subplot(4,2,1)
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,1,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'r.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,13,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'g.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,25,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'b.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,7,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'k.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,19,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'k.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,31,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'k.' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('X Position [m]')
    legend('X_1','X_2','X_3','X_{c1}','X_{c2}','X_{3com}')
    grid on
    subplot(4,2,2)
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,2,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'r.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,14,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'g.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,26,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'b.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,8,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'k.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,20,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'k.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,32,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'k.' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('X Velocity [m.s^{-1}]')
    legend('X_1','X_2','X_3','X_{c1}','X_{c2}','X_{3com}')
    grid on
    
    subplot(4,2,3)
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,3,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'r.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,15,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'g.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,27,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'b.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,9,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'k.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,21,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'k.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,33,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'k.' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('Y Position [m]')
    legend('Y_1','Y_2','Y_3','Y_{c1}','Y_{c2}','Y_{3com}')
    grid on
    
    subplot(4,2,4)
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,4,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'r.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,16,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'g.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,28,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'b.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,10,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'k.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,22,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'k.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,34,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'k.' )
    xlim([0 solution.tf])
    ylim([-1 1])
    xlabel('Time [s]')
    ylabel('Y Velocity [m.s^{-1}]')
    legend('Y_1','Y_2','Y_3','Y_{c1}','Y_{c2}','Y_{3com}')
    grid on
    
    subplot(4,2,5)
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,5,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'r.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,17,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'g.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,23,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'b.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,11,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'k.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,23,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'k.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,35,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'k.' )
    xlim([0 solution.tf])
    ylim([0 5])
    xlabel('Time [s]')
    ylabel('Z Position [m]')
    legend('Z_1','Z_2','Z_3','Z_{c1}','Z_{c2}','Z_{3com}')
    grid on
    
    subplot(4,2,6)
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,6,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'r.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,18,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'g.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,30,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'b.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,12,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'k.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,24,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'k.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,36,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'k.' )
    xlim([0 solution.tf])
    ylim([-1 1])
    xlabel('Time [s]')
    ylabel('Z Velocity [m.s^{-1}]')
    legend('Z_1','Z_2','Z_3','Z_{c1}','Z_{c2}','Z_{3com}')
    grid on
    
    subplot(4,2,7)
    plot([solution.T(:,1); solution.tf],(speval(solution.Xp,1,solution.TSeg_Bar,[solution.T(:,1); solution.tf])-speval(solution.Xp,7,solution.TSeg_Bar,[solution.T(:,1); solution.tf]))./(speval(solution.Xp,5,solution.TSeg_Bar,[solution.T(:,1); solution.tf])-speval(solution.Xp,11,solution.TSeg_Bar,[solution.T(:,1); solution.tf])),'r.' )
    hold on
    plot([solution.T(:,1); solution.tf],(speval(solution.Xp,3,solution.TSeg_Bar,[solution.T(:,1); solution.tf])-speval(solution.Xp,9,solution.TSeg_Bar,[solution.T(:,1); solution.tf]))./(speval(solution.Xp,5,solution.TSeg_Bar,[solution.T(:,1); solution.tf])-speval(solution.Xp,11,solution.TSeg_Bar,[solution.T(:,1); solution.tf])),'ro' )
    hold on
    plot([solution.T(:,1); solution.tf],(speval(solution.Xp,13,solution.TSeg_Bar,[solution.T(:,1); solution.tf])-speval(solution.Xp,19,solution.TSeg_Bar,[solution.T(:,1); solution.tf]))./(speval(solution.Xp,17,solution.TSeg_Bar,[solution.T(:,1); solution.tf])-speval(solution.Xp,23,solution.TSeg_Bar,[solution.T(:,1); solution.tf])),'g.' )
    hold on
    plot([solution.T(:,1); solution.tf],(speval(solution.Xp,15,solution.TSeg_Bar,[solution.T(:,1); solution.tf])-speval(solution.Xp,21,solution.TSeg_Bar,[solution.T(:,1); solution.tf]))./(speval(solution.Xp,17,solution.TSeg_Bar,[solution.T(:,1); solution.tf])-speval(solution.Xp,23,solution.TSeg_Bar,[solution.T(:,1); solution.tf])),'go' )
    hold on
    plot([solution.T(:,1); solution.tf],(speval(solution.Xp,25,solution.TSeg_Bar,[solution.T(:,1); solution.tf])-speval(solution.Xp,31,solution.TSeg_Bar,[solution.T(:,1); solution.tf]))./(speval(solution.Xp,29,solution.TSeg_Bar,[solution.T(:,1); solution.tf])-speval(solution.Xp,35,solution.TSeg_Bar,[solution.T(:,1); solution.tf])),'b.' )
    hold on
    plot([solution.T(:,1); solution.tf],(speval(solution.Xp,27,solution.TSeg_Bar,[solution.T(:,1); solution.tf])-speval(solution.Xp,33,solution.TSeg_Bar,[solution.T(:,1); solution.tf]))./(speval(solution.Xp,29,solution.TSeg_Bar,[solution.T(:,1); solution.tf])-speval(solution.Xp,35,solution.TSeg_Bar,[solution.T(:,1); solution.tf])),'bo' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('States')
    legend('tan(\alpha)_1','tan(\beta)_1','tan(\alpha)_2','tan(\beta)_2','tan(\alpha)_3','tan(\beta)_3')
    grid on
    
    subplot(4,2,8)
    plot([solution.T(:,1); solution.tf],speval(solution.Up,4,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'r.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Up,8,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'g.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Up,12,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'b.' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('Cable Tension [N]')
    legend('F_1','F_2','F_3')
    grid on
    
    figure('Name','Open Loop Inputs')
    subplot(3,1,1)
    plot([solution.T(:,1); solution.tf],speval(solution.Up,1,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'r.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Up,5,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'g.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Up,9,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'b.' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('X Inputs [N]')
    legend('U_{X1}','U_{X2}','U_{X3}')
    grid on
    
    subplot(3,1,2)
    plot([solution.T(:,1); solution.tf],speval(solution.Up,2,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'r.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Up,6,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'g.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Up,10,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'b.' )
    xlim([0 solution.tf])
    ylim([-1 1])
    xlabel('Time [s]')
    ylabel('Y Inputs [N]')
    legend('U_{Y1}','U_{Y2}','U_{Y3}')
    grid on
    
    subplot(3,1,3)
    plot([solution.T(:,1); solution.tf],speval(solution.Up,3,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'r.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Up,7,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'g.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Up,11,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'b.' )
    xlim([0 solution.tf])
%     ylim([26 28])
    xlabel('Time [s]')
    ylabel('Z Inputs [N]')
    legend('U_{Z1}','U_{Z2}','U_{Z3}')
    grid on    
else
    xx=linspace(solution.T(1,1),solution.T(end,1),1000);
    figure('Name','Open Loop States')
    subplot(4,2,1)
    plot(solution.T(:,1),speval(solution.Xp,1,solution.T(:,1)),'r.' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,13,solution.T(:,1)),'g.' )    
    hold on
    plot(solution.T(:,1),speval(solution.Xp,7,solution.T(:,1)),'k.' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,19,solution.T(:,1)),'k.' )
    hold on
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('X Position [m]')
    legend('X_1','X_2','X_{c1}','X_{c2}')
    grid on
    
    subplot(4,2,2)
    plot(solution.T(:,1),speval(solution.Xp,2,solution.T(:,1)),'r.' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,14,solution.T(:,1)),'g.' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,8,solution.T(:,1)),'k.' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,20,solution.T(:,1)),'k.' )
    hold on
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('X Velocity [m.s^{-1}]')
    legend('X_1','X_2','X_{c1}','X_{c2}')
    grid on
    
    subplot(4,2,3)
    plot(solution.T(:,1),speval(solution.Xp,3,solution.T(:,1)),'r.' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,15,solution.T(:,1)),'g.' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,9,solution.T(:,1)),'k.' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,21,solution.T(:,1)),'k.' )
    hold on
    xlim([0 solution.tf])
    ylim([-1 1])
    xlabel('Time [s]')
    ylabel('Y Position [m]')
    legend('Y_1','Y_2','Y_{c1}','Y_{c2}')
    grid on
    
    subplot(4,2,4)
    plot(solution.T(:,1),speval(solution.Xp,4,solution.T(:,1)),'r.' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,16,solution.T(:,1)),'g.' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,10,solution.T(:,1)),'k.' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,22,solution.T(:,1)),'k.' )
    hold on
    xlim([0 solution.tf])
    ylim([-1 1])
    xlabel('Time [s]')
    ylabel('Y Velocity [m.s^{-1}]')
    legend('Y_1','Y_2','Y_{c1}','Y_{c2}')
    grid on
    
    subplot(4,2,5)
    plot(solution.T(:,1),speval(solution.Xp,5,solution.T(:,1)),'r.' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,17,solution.T(:,1)),'g.' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,11,solution.T(:,1)),'k.' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,23,solution.T(:,1)),'k.' )
    hold on
    
%     x_start = 3.7;
%     x_end = 4.3;
%     spline1 = speval(solution.Xp,7,xx);
%     obstacle_time_start1 = xx(find(spline1>x_start,1));
%     obstacle_time_end1 = xx(find(spline1>x_end,1));
%     
%     spline2 = speval(solution.Xp,19,xx);
%     obstacle_time_start2 = xx(find(spline2>x_start,1));
%     obstacle_time_end2 = xx(find(spline2>x_end,1));
%     obs_height=1.5;
%     plot(xx,obs_height.*(tanh((xx-obstacle_time_start1)) - tanh((xx-obstacle_time_end1))))
%     hold on 
%     plot(xx,obs_height.*(tanh((xx-obstacle_time_start2)) - tanh((xx-obstacle_time_end2))))
    xlim([0 solution.tf])
%     ylim([1 3])
    xlabel('Time [s]')
    ylabel('Z Position [m]')
    legend('Z_1','Z_2','Z_{c1}','Z_{c2}');%,'Obstacle #1','Obstacle #2')
    grid on
    
    subplot(4,2,6)
    plot(solution.T(:,1),speval(solution.Xp,6,solution.T(:,1)),'r.' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,18,solution.T(:,1)),'g.' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,12,solution.T(:,1)),'k.' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,24,solution.T(:,1)),'k.' )
    hold on
    xlim([0 solution.tf])
    ylim([-1 1])
    xlabel('Time [s]')
    ylabel('Z Velocity [m.s^{-1}]')
    legend('Z_1','Z_2','Z_{c1}','Z_{c2}')
    grid on
    
    subplot(4,2,7)
    plot(solution.T(:,1),(speval(solution.Xp,1,solution.T(:,1))-speval(solution.Xp,7,solution.T(:,1)))./(speval(solution.Xp,5,solution.T(:,1))-speval(solution.Xp,11,solution.T(:,1))),'rx' )
    hold on
    plot(solution.T(:,1),(speval(solution.Xp,3,solution.T(:,1))-speval(solution.Xp,9,solution.T(:,1)))./(speval(solution.Xp,5,solution.T(:,1))-speval(solution.Xp,11,solution.T(:,1))),'gx' )
    hold on
    plot(solution.T(:,1),(speval(solution.Xp,13,solution.T(:,1))-speval(solution.Xp,19,solution.T(:,1)))./(speval(solution.Xp,17,solution.T(:,1))-speval(solution.Xp,23,solution.T(:,1))),'ro' )
    hold on
    plot(solution.T(:,1),(speval(solution.Xp,15,solution.T(:,1))-speval(solution.Xp,21,solution.T(:,1)))./(speval(solution.Xp,17,solution.T(:,1))-speval(solution.Xp,23,solution.T(:,1))),'go' )
    hold on
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('States')
    legend('tan(\alpha)_1','tan(\beta)_1','tan(\alpha)_2','tan(\beta)_2')
    grid on
    
    subplot(4,2,8)
    plot(solution.T(:,1),speval(solution.Up,4,solution.T(:,1)),'r.' )
    hold on
    plot(solution.T(:,1),speval(solution.Up,8,solution.T(:,1)),'g.' )
    hold on
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('Cable Tension [N]')
    legend('F_1','F_2')
    grid on
    
    figure('Name','Open Loop Inputs')
    subplot(3,1,1)
    plot(solution.T(:,1),speval(solution.Up,1,solution.T(:,1)),'r.' )
    hold on
    plot(solution.T(:,1),speval(solution.Up,5,solution.T(:,1)),'g.' )
%     hold on
%     plot(solution.T(:,1),speval(solution.Up,1,solution.T(:,1))+speval(solution.Up,4,solution.T(:,1))+speval(solution.Up,7,solution.T(:,1)),'k.' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('X Inputs [N]')
    legend('U_{X1}','U_{X2}');%,'Sum of U_{Xi}')
    grid on
    subplot(3,1,2)
    plot(solution.T(:,1),speval(solution.Up,2,solution.T(:,1)),'r.' )
    hold on
    plot(solution.T(:,1),speval(solution.Up,6,solution.T(:,1)),'g.' )
    hold on
    xlim([0 solution.tf])
    ylim([-1 1])
    xlabel('Time [s]')
    ylabel('Y Inputs [N]')
    legend('U_{Y1}','U_{Y2}')
    grid on
    subplot(3,1,3)
    plot(solution.T(:,1),speval(solution.Up,3,solution.T(:,1)),'r.' )
    hold on
    plot(solution.T(:,1),speval(solution.Up,7,solution.T(:,1)),'g.' )
    hold on
    xlim([0 solution.tf])
%     ylim([14 15])
    xlabel('Time [s]')
    ylabel('Z Inputs [N]')
    legend('U_{Z1}','U_{Z2}')
    grid on
end