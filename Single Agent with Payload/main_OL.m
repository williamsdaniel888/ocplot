% main_OL - Script for open-loop simulation using ICLOCS2
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

clc;
clear all;
close all;
format compact;
addpath(genpath('..\ICLOCS\src'))
addpath(genpath('..\Ipopt'))
global sol;  
sol=[];                             % Initialize solution structure

options= settings_hscubconst(15);            % Get options and solver settings 
[problem,guess]=MAXProblem;    % Fetch the problem definition
% 
tic
[infoNLP,data,options]=transcribeOCP(problem,guess,options); % Format for NLP solver
[solution,status,data] = solveNLP(infoNLP,data);      % Solve the NLP
[solution]=output(problem,solution,options,data,4);         % Output solutions
fprintf('Total computation time: %s s\n',toc)
% 
% errorHistory=zeros(2,length(problem.states.x0));
% npsegmentHistory=zeros(2,1);
% ConstraintErrorHistory=zeros(2,length(problem.constraintErrorTol));
% timeHistory=zeros(1,2);
% iterHistory=zeros(1,2);
% solutionHistory=cell(1,2);
% 
% maxAbsError=1e9;
% i=1; imax=50;
% tic;
% while (any(maxAbsError>problem.states.xErrorTol) || any(maxAbsConstraintError>problem.constraintErrorTol)) && i<=imax    
%     [infoNLP,data,options]=transcribeOCP(problem,guess,options); % Format for NLP solver
%     [solution,status,data] = solveNLP(infoNLP,data);      % Solve the NLP
%     [solution]=output(problem,solution,options,data,4);         % Output solutions
%     
%     
%     maxAbsError=max(abs(solution.Error));
%     maxAbsConstraintError=max(solution.ConstraintError);
%     errorHistory(i,:)=maxAbsError;
%     iterHistory(i)=status.iter;
%     ConstraintErrorHistory(i,:)=maxAbsConstraintError;
%     timeHistory(i)=solution.computation_time;
%     solutionHistory{i}=solution;
%     
%     if (any(maxAbsError>problem.states.xErrorTol) || any(maxAbsConstraintError>problem.constraintErrorTol)) && i<=imax
%         [ options, guess ] = doMeshRefinement( options, problem, guess, data, solution, i );
%     end
%     i=i+1;
% 
% end
% fprintf('Total computation time: %s s\n',toc)
% MeshRefinementHistory.errorHistory=errorHistory;
% MeshRefinementHistory.timeHistory=timeHistory;
% MeshRefinementHistory.iterHistory=iterHistory;
% MeshRefinementHistory.ConstraintErrorHistory=ConstraintErrorHistory;

xx=linspace(solution.T(1,1),solution.tf,1000);

if (strcmp(options.transcription,'globalLGR')) || (strcmp(options.transcription,'hpLGR'))

    figure('Name','Open Loop States')
    subplot(4,2,1)
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,1,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'r.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,7,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'g.' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('X Position [m]')
    legend('X_1','X_{c1}')
    grid on
    subplot(4,2,2)
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,2,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'m.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,8,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'b.' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('X Velocity [m.s^{-1}]')
    legend('X_1','X_{c1}')
    grid on
    
    subplot(4,2,3)
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,3,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'r.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,9,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'g.' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('Y Position [m]')
    legend('Y_1','Y_{c1}')
    grid on
    subplot(4,2,4)
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,4,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'m.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,10,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'b.' )
    xlim([0 solution.tf])
    ylim([-1 1])
    xlabel('Time [s]')
    ylabel('Y Velocity [m.s^{-1}]')
    legend('Y_1','Y_{c1}')
    grid on
    
    subplot(4,2,5)
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,5,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'r.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,11,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'g.' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('Z Position [m]')
    legend('Z_1','Z_{c1}')
    grid on
    
    subplot(4,2,6)
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,6,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'m.' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,12,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'b.' )
    xlim([0 solution.tf])
    ylim([-1 1])
    xlabel('Time [s]')
    ylabel('Z Velocity [m.s^{-1}]')
    legend('Z_1','Z_{c1}')
    grid on
    
    subplot(4,2,7)
    plot([solution.T(:,1); solution.tf],(speval(solution.Xp,1,solution.TSeg_Bar,[solution.T(:,1); solution.tf])-speval(solution.Xp,7,solution.TSeg_Bar,[solution.T(:,1); solution.tf]))./(speval(solution.Xp,5,solution.TSeg_Bar,[solution.T(:,1); solution.tf])-speval(solution.Xp,11,solution.TSeg_Bar,[solution.T(:,1); solution.tf])),'r.' )
    hold on
    plot([solution.T(:,1); solution.tf],(speval(solution.Xp,3,solution.TSeg_Bar,[solution.T(:,1); solution.tf])-speval(solution.Xp,9,solution.TSeg_Bar,[solution.T(:,1); solution.tf]))./(speval(solution.Xp,5,solution.TSeg_Bar,[solution.T(:,1); solution.tf])-speval(solution.Xp,11,solution.TSeg_Bar,[solution.T(:,1); solution.tf])),'g.' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('States')
    legend('tan(\alpha)','tan(\beta)')
    grid on
    
    subplot(4,2,8)
    plot([solution.T(:,1); solution.tf],speval(solution.Up,4,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'b.' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('Cable Tension [N]')
    grid on
    
    figure('Name','Open Loop Inputs')
    subplot(3,1,1)
    plot([solution.T(:,1); solution.tf],speval(solution.Up,1,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'r.' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('U_{X1} [N]')
    grid on
    subplot(3,1,2)
    plot([solution.T(:,1); solution.tf],speval(solution.Up,2,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'g.' )
    xlim([0 solution.tf])
    ylim([-1 1])
    xlabel('Time [s]')
    ylabel('U_{Y1} [N]')
    grid on
    subplot(3,1,3)
    plot([solution.T(:,1); solution.tf],speval(solution.Up,3,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'b.' )
    xlim([0 solution.tf])
    ylim([26 28])
    xlabel('Time [s]')
    ylabel('U_{Z1} [N]')
    grid on    
else
    xx=linspace(solution.T(1,1),solution.T(end,1),1000);
    figure('Name','Open Loop States')
    subplot(4,2,1)
    plot(solution.T(:,1),speval(solution.Xp,1,solution.T(:,1)),'r.' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,7,solution.T(:,1)),'g.' )
    hold on
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('X Position [m]')
    legend('X_1','X_{c1}')
    grid on
    
    subplot(4,2,2)
    plot(solution.T(:,1),speval(solution.Xp,2,solution.T(:,1)),'m.' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,8,solution.T(:,1)),'b.' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('X Velocity [m.s^{-1}]')
    legend('X_1','X_{c1}')
    grid on
    
    subplot(4,2,3)
    plot(solution.T(:,1),speval(solution.Xp,3,solution.T(:,1)),'r.' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,9,solution.T(:,1)),'g.' )
    hold on
    xlim([0 solution.tf])
    ylim([-1 1])
    xlabel('Time [s]')
    ylabel('Y Position [m]')
    legend('Y_1','Y_{c1}')
    grid on
    
    subplot(4,2,4)
    plot(solution.T(:,1),speval(solution.Xp,4,solution.T(:,1)),'m.' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,10,solution.T(:,1)),'b.' )
    xlim([0 solution.tf])
    ylim([-1 1])
    xlabel('Time [s]')
    ylabel('Y Velocity [m.s^{-1}]')
    legend('Y_1','Y_{c1}')
    grid on
    
    subplot(4,2,5)
    plot(solution.T(:,1),speval(solution.Xp,5,solution.T(:,1)),'r.' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,11,solution.T(:,1)),'g.' )
    hold on
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('Z Position [m]')
    legend('Z_1','Z_{c1}')
    grid on
    
    subplot(4,2,6)
    plot(solution.T(:,1),speval(solution.Xp,6,solution.T(:,1)),'m.' )
    hold on
    plot(solution.T(:,1),speval(solution.Xp,12,solution.T(:,1)),'b.' )
    xlim([0 solution.tf])
%     ylim([-1 1])
    xlabel('Time [s]')
    ylabel('Z Velocity [m.s^{-1}]')
    legend('Z_1','Z_{c1}')
    grid on
    
    subplot(4,2,7)
    plot(solution.T(:,1),(speval(solution.Xp,1,solution.T(:,1))-speval(solution.Xp,7,solution.T(:,1)))./(speval(solution.Xp,5,solution.T(:,1))-speval(solution.Xp,11,solution.T(:,1))),'r.' )
    hold on
    plot(solution.T(:,1),(speval(solution.Xp,3,solution.T(:,1))-speval(solution.Xp,9,solution.T(:,1)))./(speval(solution.Xp,5,solution.T(:,1))-speval(solution.Xp,11,solution.T(:,1))),'g.' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('States')
    legend('tan(\alpha)','tan(\beta)')
    grid on
    
    subplot(4,2,8)
    plot(solution.T(:,1),speval(solution.Up,4,solution.T(:,1)),'m.' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('Cable Tension [N]')
    grid on
    
    figure('Name','Open Loop Inputs')
    subplot(3,1,1)
    plot(solution.T(:,1),speval(solution.Up,1,solution.T(:,1)),'r.' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    ylabel('U_{X1} [N]')
    grid on
    subplot(3,1,2)
    plot(solution.T(:,1),speval(solution.Up,2,solution.T(:,1)),'g.' )
    xlim([0 solution.tf])
    ylim([-1 1])
    xlabel('Time [s]')
    ylabel('U_{Y1} [N]')
    grid on
    subplot(3,1,3)
    plot(solution.T(:,1),speval(solution.Up,3,solution.T(:,1)),'b.' )
    xlim([0 solution.tf])
%     ylim([26 28])
    xlabel('Time [s]')
    ylabel('U_{Z1} [N]')
    grid on

end
ol_data_x =timeseries(speval(solution.Up,1,solution.T(:,1)),solution.T(:,1));
ol_data_y = timeseries(speval(solution.Up,2,solution.T(:,1)),solution.T(:,1));
ol_data_z = timeseries(speval(solution.Up,3,solution.T(:,1)),solution.T(:,1));
ol_data_F = timeseries(speval(solution.Up,4,solution.T(:,1)),solution.T(:,1));
save('matlab.mat','ol_data_x','ol_data_y','ol_data_z','ol_data_F')