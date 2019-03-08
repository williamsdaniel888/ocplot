% main_Auto_MeshRefinement - Main script to solve the Optimal Control Problem with automatic mesh selection and refinement
%
% Multi-Agent QUAV Control (Double Integrator Minimum Actuator Effort Squared Repositioning) Problem
%
% Copyright (C) 2018 Yuanbo Nie, Omar Faqir, and Eric Kerrigan. All Rights Reserved.
% The contribution of Paola Falugi, Eric Kerrigan and Eugene van Wyk for the work on ICLOCS Version 1 (2010) is kindly acknowledged.
% This code is published under the BSD License.
% Department of Aeronautics and Department of Electrical and Electronic Engineering,
% Imperial College London London  England, UK 
% ICLOCS (Imperial College London Optimal Control) Version 2.0 
% 1 May 2018
% iclocs@imperial.ac.uk


%--------------------------------------------------------

clc;clear all;close all;format compact;
global sol;  
sol=[];                             % Initialize solution structure

options= settings_h(10);                  % Get options and solver settings 
[problem,guess]=MultiAgentQUAVProblemNewtRestricted;%SingleAgentQUAVProblem;          % Fetch the problem definition

[infoNLP,data,options]=transcribeOCP(problem,guess,options); % Format for NLP solver
[solution,status,data] = solveNLP(infoNLP,data);      % Solve the NLP
[solution]=output(problem,solution,options,data,4);         % Output solutions

%% BE CAREFUL; CODE BELOW HERE MUST BE REFACTORED FOR MULTIAGENT STATES
xx=linspace(solution.T(1,1),solution.tf,1000);

if (strcmp(options.transcription,'globalLGR')) || (strcmp(options.transcription,'hpLGR'))

    figure
    subplot(3,1,1)
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,1,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'ro' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,2,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'md' )
    plot(xx,speval(solution.Xp,1,solution.TSeg_Bar,xx),'r-' )
    plot(xx,speval(solution.Xp,2,solution.TSeg_Bar,xx),'m-' )
    xlabel('Time [s]')
    ylabel('States')
    legend('X Position [m]','X Velocity [m/s]')
    grid on
    
    subplot(3,1,2)
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,3,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'go' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,4,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'yd' )
    plot(xx,speval(solution.Xp,3,solution.TSeg_Bar,xx),'g-' )
    plot(xx,speval(solution.Xp,4,solution.TSeg_Bar,xx),'y-' )
    xlabel('Time [s]')
    ylabel('States')
    legend('Y Position [m]','Y Velocity [m/s]')
    grid on
    
    subplot(3,1,3)
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,5,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'bo' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,6,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'cd' )
    plot(xx,speval(solution.Xp,3,solution.TSeg_Bar,xx),'b-' )
    plot(xx,speval(solution.Xp,4,solution.TSeg_Bar,xx),'c-' )
    xlabel('Time [s]')
    ylabel('States')
    legend('Z Position [m]','Z Velocity [m/s]')
    grid on
    
    figure
    subplot(3,1,1)
    plot([solution.T(:,1); solution.tf],speval(solution.Up,1,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'ko' )
    hold on
    plot([solution.T(1,1); solution.tf],[problem.inputs.ul(1), problem.inputs.ul(1)],'r-' )
    plot([solution.T(1,1); solution.tf],[problem.inputs.uu(1), problem.inputs.uu(1)],'r-' )
    plot(xx,speval(solution.Up,1,solution.TSeg_Bar,xx),'b-' )
    xlabel('Time [s]')
    grid on
    ylabel('Control Input')
    legend('u_X [N]')
    subplot(3,1,2)
    plot([solution.T(:,1); solution.tf],speval(solution.Up,2,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'ko' )
    hold on
    plot([solution.T(1,1); solution.tf],[problem.inputs.ul(2), problem.inputs.ul(2)],'r-' )
    plot([solution.T(1,1); solution.tf],[problem.inputs.uu(2), problem.inputs.uu(2)],'r-' )
    plot(xx,speval(solution.Up,2,solution.TSeg_Bar,xx),'b-' )
    xlabel('Time [s]')
    grid on
    ylabel('Control Input')
    legend('u_Y [N]')
    subplot(3,1,3)
    plot([solution.T(:,1); solution.tf],speval(solution.Up,3,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'ko' )
    hold on
    plot([solution.T(1,1); solution.tf],[problem.inputs.ul(3), problem.inputs.ul(3)],'r-' )
    plot([solution.T(1,1); solution.tf],[problem.inputs.uu(3), problem.inputs.uu(3)],'r-' )
    plot(xx,speval(solution.Up,3,solution.TSeg_Bar,xx),'b-' )
    xlabel('Time [s]')
    grid on
    ylabel('Control Input')
    legend('u_Z [N]')
else
    xx=linspace(solution.T(1,1),solution.T(end,1),1000);
    figure
    subplot(5,1,1)
    plot(solution.T(:,1),speval(solution.Xp,1,solution.T(:,1)),'ro' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,7,solution.T(:,1)),'mo' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,13,solution.T(:,1)),'go' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,19,solution.T(:,1)),'bo' )
    plot(xx,speval(solution.Xp,1,xx),'r-.' )
    plot(xx,speval(solution.Xp,7,xx),'m-' )
    plot(xx,speval(solution.Xp,13,xx),'g-.' )
    plot(xx,speval(solution.Xp,19,xx),'b-.' )
    ylim([0 8])
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('States')
    legend('X_1 Position [m]','X_{com} Position [m]','X_2 Position [m]','X_3 Position [m]')
    grid on
    subplot(5,1,2)
    plot(solution.T(:,1),speval(solution.Xp,3,solution.T(:,1)),'ro' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,9,solution.T(:,1)),'mo' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,15,solution.T(:,1)),'go' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,21,solution.T(:,1)),'bo' )
    plot(xx,speval(solution.Xp,3,xx),'r-.' )
    plot(xx,speval(solution.Xp,9,xx),'m-' )
    plot(xx,speval(solution.Xp,15,xx),'g-.' )
    plot(xx,speval(solution.Xp,21,xx),'b-.' )
    ylim([-1 1])
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('States')
    legend('Y_1 Position [m]','Y_{com} Position [m]','Y_2 Position [m]','Y_3 Position [m]')
    grid on
    subplot(5,1,3)
    plot(solution.T(:,1),speval(solution.Xp,5,solution.T(:,1)),'ro' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,11,solution.T(:,1)),'mo' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,17,solution.T(:,1)),'go' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,23,solution.T(:,1)),'bo' )
    plot(xx,speval(solution.Xp,5,xx),'r-.' )
    plot(xx,speval(solution.Xp,11,xx),'m-' )
    plot(xx,speval(solution.Xp,17,xx),'g-.' )
    plot(xx,speval(solution.Xp,23,xx),'b-.' )
    ylim([0 5])
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('States')
    legend('Z_1 Position [m]','Z_{com} Position [m]','Z_2 Position [m]','Z_3 Position [m]')
    grid on
    subplot(5,1,4)
    plot(solution.T(:,1),speval(solution.Up,4,solution.T),'r-o' )
    hold on
    plot(solution.T(:,1),speval(solution.Up,5,solution.T),'g-o' )
    hold on
    plot(solution.T(:,1),speval(solution.Up,6,solution.T),'b-o' )
    hold on
%     plot([solution.T(1,1); solution.tf],[problem.inputs.ul(3), problem.inputs.ul(3)],'r-' )
%     plot([solution.T(1,1); solution.tf],[problem.inputs.uu(3), problem.inputs.uu(3)],'r-' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    grid on
    ylabel('Cable Tension')
    legend('F_{1} [N]','F_{2} [N]','F_{3} [N]')
        subplot(5,1,5)
    plot(solution.T(:,1),speval(solution.Up,7,solution.T),'r-.o' )
    hold on
    plot(solution.T(:,1),speval(solution.Up,8,solution.T),'g-.o' )
    hold on
    plot(solution.T(:,1),speval(solution.Up,9,solution.T),'b-.o' )
    hold on
    plot(solution.T(:,1),speval(solution.Up,10,solution.T),'c-.o' )
    hold on
    plot(solution.T(:,1),speval(solution.Up,11,solution.T),'y-.o' )
    hold on
    plot(solution.T(:,1),speval(solution.Up,12,solution.T),'m-.o' )
    hold on
%     plot([solution.T(1,1); solution.tf],[problem.inputs.ul(3), problem.inputs.ul(3)],'r-' )
%     plot([solution.T(1,1); solution.tf],[problem.inputs.uu(3), problem.inputs.uu(3)],'r-' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    grid on
    ylabel('Cable Angle')
    legend('tan(\alpha_{1})','tan(\beta_{1})','tan(\alpha_{2})','tan(\beta_{2})','tan(\alpha_{3})','tan(\beta_{3})')
    
    figure
    subplot(3,1,1)
    plot(solution.T(:,1),speval(solution.Up,1,solution.T),'r-o' )
    hold on
    plot(solution.T(:,1),speval(solution.Up,13,solution.T),'g-o' )
    hold on
    plot(solution.T(:,1),speval(solution.Up,17,solution.T),'b-o' )
    hold on
%     plot([solution.T(1,1); solution.tf],[problem.inputs.ul(1), problem.inputs.ul(1)],'r-' )
%     plot([solution.T(1,1); solution.tf],[problem.inputs.uu(1), problem.inputs.uu(1)],'r-' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    grid on
    ylabel('Control Input')
    legend('u_{X1} [N]','u_{X2} [N]','u_{X3} [N]')
    subplot(3,1,2)
    plot(solution.T(:,1),speval(solution.Up,2,solution.T),'r-o' )
    hold on
    plot(solution.T(:,1),speval(solution.Up,14,solution.T),'g-o' )
    hold on
    plot(solution.T(:,1),speval(solution.Up,17,solution.T),'b-o' )
    hold on
%     plot([solution.T(1,1); solution.tf],[problem.inputs.ul(2), problem.inputs.ul(2)],'r-' )
%     plot([solution.T(1,1); solution.tf],[problem.inputs.uu(2), problem.inputs.uu(2)],'r-' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    grid on
    ylabel('Control Input')
    legend('u_{Y1} [N]','u_{Y2} [N]','u_{Y3} [N]')
    subplot(3,1,3)
    plot(solution.T(:,1),speval(solution.Up,3,solution.T),'r-o' )
    hold on
    plot(solution.T(:,1),speval(solution.Up,15,solution.T),'g-o' )
    hold on
    plot(solution.T(:,1),speval(solution.Up,18,solution.T),'b-o' )
    hold on
%     plot([solution.T(1,1); solution.tf],[problem.inputs.ul(3), problem.inputs.ul(3)],'r-' )
%     plot([solution.T(1,1); solution.tf],[problem.inputs.uu(3), problem.inputs.uu(3)],'r-' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    grid on
    ylabel('Control Input')
    legend('u_{Z1} [N]','u_{Z2} [N]','u_{Z3} [N]')
end