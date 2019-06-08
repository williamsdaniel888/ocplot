% main_OLStats - Script for collecting statistics for the open-loop
% simulations
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
N = [10,20,40,60,100];
comp_time = zeros(3,5);
max_error = zeros(3,5);
max_rel_error = zeros(3,5);
max_cst_error = zeros(3,5);

for i = 1:length(N)
    options= settings_euler(N(i));            % Get options and solver settings 
    [problem,guess]= MAGProblem;%    % Fetch the problem definition
    % 
    tic
    [infoNLP,data,options]=transcribeOCP(problem,guess,options); % Format for NLP solver
    [solution_EUL,status,data] = solveNLP(infoNLP,data);      % Solve the NLP
    [solution_EUL]=output(problem,solution_EUL,options,data,0);         % Output solutions
    comp_time(1,i)=toc;
    max_error(1,i) = max(max(solution_EUL.Error));
    max_rel_error(1,i) = max(max(solution_EUL.ErrorRelative));
    max_cst_error(1,i) = max(max(solution_EUL.ConstraintError));

    sol=[];                             % Initialize solution structure

    options= settings_trap(N(i));            % Get options and solver settings 
    [problem,guess]=MAGProblem;%SingleAgentQUAVProblem;%    % Fetch the problem definition
    % 
    tic
    [infoNLP,data,options]=transcribeOCP(problem,guess,options); % Format for NLP solver
    [solution_TRAP,status,data] = solveNLP(infoNLP,data);      % Solve the NLP
    [solution_TRAP]=output(problem,solution_TRAP,options,data,0);         % Output solutions
    comp_time(2,i)=toc;
    max_error(2,i) = max(max(solution_TRAP.Error));
    max_rel_error(2,i) = max(max(solution_TRAP.ErrorRelative));
    max_cst_error(2,i) = max(max(solution_TRAP.ConstraintError));

    sol=[];                             % Initialize solution structure

    options= settings_hscubconst(N(i));            % Get options and solver settings 
    [problem,guess]=MAGProblem;%SingleAgentQUAVProblem;%    % Fetch the problem definition
    % 
    tic
    [infoNLP,data,options]=transcribeOCP(problem,guess,options); % Format for NLP solver
    [solution_HSCUBCONST,status,data] = solveNLP(infoNLP,data);      % Solve the NLP
    [solution_HSCUBCONST]=output(problem,solution_HSCUBCONST,options,data,0);         % Output solutions
    comp_time(3,i)=toc;
    max_error(3,i) = max(max(solution_HSCUBCONST.Error));
    max_rel_error(3,i) = max(max(solution_HSCUBCONST.ErrorRelative));
    max_cst_error(3,i) = max(max(solution_HSCUBCONST.ConstraintError));

    sol=[];                             % Initialize solution structure
end

figure('Name','Maximum Error')
plot(log10(N),log10(max_error(1,:)),'rx')
hold on
plot(log10(N),log10(max_error(2,:)),'gx')
hold on
plot(log10(N),log10(max_error(3,:)),'bx')
xlabel('log_{10} Mesh Points')
ylabel('log_{10} Maximum Error')
legend('Euler','Trapezoidal','HS')
figure('Name','Maximum Relative Error')
plot(log10(N),log10(max_rel_error(1,:)),'rx')
hold on
plot(log10(N),log10(max_rel_error(2,:)),'gx')
hold on
plot(log10(N),log10(max_rel_error(3,:)),'bx')
xlabel('log_{10} Mesh Points')
ylabel('log_{10} Maximum Relative Error')
legend('Euler','Trapezoidal','HS')
figure('Name','Computation Time')
plot(log10(N),log10(comp_time(1,:)),'rx')
hold on
plot(log10(N),log10(comp_time(2,:)),'gx')
hold on
plot(log10(N),log10(comp_time(3,:)),'bx')
xlabel('log_{10} Mesh Points')
ylabel('log_{10} Computation Time [s]')
legend('Euler','Trapezoidal','HS')
figure('Name','Computation Time vs Maximum Error Tradeoff')
plot(log10(comp_time(1,:)),log10(max_error(1,:)),'rx')
hold on
plot(log10(comp_time(2,:)),log10(max_error(2,:)),'gx')
hold on
plot(log10(comp_time(3,:)),log10(max_error(3,:)),'bx')
xlabel('log_{10} Computation Time [s]')
ylabel('log_{10} Maximum Error')
legend('Euler','Trapezoidal','HS')
figure('Name','Computation Time vs Maximum Relative Error Tradeoff')
plot(log10(comp_time(1,:)),log10(max_rel_error(1,:)),'rx')
hold on
plot(log10(comp_time(2,:)),log10(max_rel_error(2,:)),'gx')
hold on
plot(log10(comp_time(3,:)),log10(max_rel_error(3,:)),'bx')
xlabel('log_{10} Computation Time [s]')
ylabel('log_{10} Maximum Relative Error')
legend('Euler','Trapezoidal','HS')

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
% fprintf('Total computation time: %s s\Mesh Points',toc)
% MeshRefinementHistory.errorHistory=errorHistory;
% MeshRefinementHistory.timeHistory=timeHistory;
% MeshRefinementHistory.iterHistory=iterHistory;
% MeshRefinementHistory.ConstraintErrorHistory=ConstraintErrorHistory;

% % xx=linspace(solution.T(1,1),solution.tf,1000);
% % 
% % if (strcmp(options.transcription,'globalLGR')) || (strcmp(options.transcription,'hpLGR'))
% % 
% %     figure('Name','Open Loop States')
% %     subplot(4,2,1)
% %     plot([solution.T(:,1); solution.tf],speval(solution.Xp,1,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'rx' )
% %     hold on
% %     plot([solution.T(:,1); solution.tf],speval(solution.Xp,7,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'gx' )
% %     xlim([0 solution.tf])
% %     xlabel('Time [s]')
% %     ylabel('X Position [m]')
% %     legend('X_1','X_{com}')
% %     grid on
% %     subplot(4,2,2)
% %     plot([solution.T(:,1); solution.tf],speval(solution.Xp,2,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'mx' )
% %     hold on
% %     plot([solution.T(:,1); solution.tf],speval(solution.Xp,8,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'bx' )
% %     xlim([0 solution.tf])
% %     xlabel('Time [s]')
% %     ylabel('X Velocity [m/s]')
% %     legend('X_1','X_{com}')
% %     grid on
% %     
% %     subplot(4,2,3)
% %     plot([solution.T(:,1); solution.tf],speval(solution.Xp,3,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'rx' )
% %     hold on
% %     plot([solution.T(:,1); solution.tf],speval(solution.Xp,9,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'gx' )
% %     xlim([0 solution.tf])
% %     xlabel('Time [s]')
% %     ylabel('Y Position [m]')
% %     legend('Y_1','Y_{com}')
% %     grid on
% %     subplot(4,2,4)
% %     plot([solution.T(:,1); solution.tf],speval(solution.Xp,4,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'mx' )
% %     hold on
% %     plot([solution.T(:,1); solution.tf],speval(solution.Xp,10,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'bx' )
% %     xlim([0 solution.tf])
% %     ylim([-1 1])
% %     xlabel('Time [s]')
% %     ylabel('Y Velocity [m/s]')
% %     legend('Y_1','Y_{com}')
% %     grid on
% %     
% %     subplot(4,2,5)
% %     plot([solution.T(:,1); solution.tf],speval(solution.Xp,5,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'rx' )
% %     hold on
% %     plot([solution.T(:,1); solution.tf],speval(solution.Xp,11,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'gx' )
% %     xlim([0 solution.tf])
% %     xlabel('Time [s]')
% %     ylabel('Z Position [m]')
% %     legend('Z_1','Z_{com}')
% %     grid on
% %     
% %     subplot(4,2,6)
% %     plot([solution.T(:,1); solution.tf],speval(solution.Xp,6,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'mx' )
% %     hold on
% %     plot([solution.T(:,1); solution.tf],speval(solution.Xp,12,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'bx' )
% %     xlim([0 solution.tf])
% %     ylim([-1 1])
% %     xlabel('Time [s]')
% %     ylabel('Z Velocity [m/s]')
% %     legend('Z_1','Z_{com}')
% %     grid on
% %     
% %     subplot(4,2,7)
% %     plot([solution.T(:,1); solution.tf],(speval(solution.Xp,1,solution.TSeg_Bar,[solution.T(:,1); solution.tf])-speval(solution.Xp,7,solution.TSeg_Bar,[solution.T(:,1); solution.tf]))./(speval(solution.Xp,5,solution.TSeg_Bar,[solution.T(:,1); solution.tf])-speval(solution.Xp,11,solution.TSeg_Bar,[solution.T(:,1); solution.tf])),'rx' )
% %     hold on
% %     plot([solution.T(:,1); solution.tf],(speval(solution.Xp,3,solution.TSeg_Bar,[solution.T(:,1); solution.tf])-speval(solution.Xp,9,solution.TSeg_Bar,[solution.T(:,1); solution.tf]))./(speval(solution.Xp,5,solution.TSeg_Bar,[solution.T(:,1); solution.tf])-speval(solution.Xp,11,solution.TSeg_Bar,[solution.T(:,1); solution.tf])),'rx' )
% %     xlim([0 solution.tf])
% %     xlabel('Time [s]')
% %     ylabel('States')
% %     legend('tan(\alpha)','tan(\beta)')
% %     grid on
% %     
% %     subplot(4,2,8)
% %     plot([solution.T(:,1); solution.tf],speval(solution.Up,4,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'bx' )
% %     xlim([0 solution.tf])
% %     xlabel('Time [s]')
% %     ylabel('Cable Tension [N]')
% %     grid on
% %     
% %     figure('Name','Open Loop Inputs')
% %     subplot(3,1,1)
% %     plot([solution.T(:,1); solution.tf],speval(solution.Up,1,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'rx' )
% %     xlim([0 solution.tf])
% %     xlabel('Time [s]')
% %     ylabel('U_{X1} [N]')
% %     grid on
% %     subplot(3,1,2)
% %     plot([solution.T(:,1); solution.tf],speval(solution.Up,2,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'gx' )
% %     xlim([0 solution.tf])
% %     ylim([-1 1])
% %     xlabel('Time [s]')
% %     ylabel('U_{Y1} [N]')
% %     grid on
% %     subplot(3,1,3)
% %     plot([solution.T(:,1); solution.tf],speval(solution.Up,3,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'bx' )
% %     xlim([0 solution.tf])
% %     ylim([26 28])
% %     xlabel('Time [s]')
% %     ylabel('U_{Z1} [N]')
% %     grid on    
% % else
% %     xx=linspace(solution.T(1,1),solution.T(end,1),1000);
% %     figure('Name','Open Loop States')
% %     subplot(4,2,1)
% %     plot(solution.T(:,1),speval(solution.Xp,1,solution.T(:,1)),'rx' )
% %     hold on
% %     plot(solution.T(:,1),speval(solution.Xp,7,solution.T(:,1)),'gx' )
% %     hold on
% %     xlim([0 solution.tf])
% %     xlabel('Time [s]')
% %     ylabel('X Position [m]')
% %     legend('X_1','X_{com}')
% %     grid on
% %     
% %     subplot(4,2,2)
% %     plot(solution.T(:,1),speval(solution.Xp,2,solution.T(:,1)),'mx' )
% %     hold on
% %     plot(solution.T(:,1),speval(solution.Xp,8,solution.T(:,1)),'bx' )
% %     xlim([0 solution.tf])
% %     xlabel('Time [s]')
% %     ylabel('X Velocity [m/s]')
% %     legend('X_1','X_{com}')
% %     grid on
% %     
% %     subplot(4,2,3)
% %     plot(solution.T(:,1),speval(solution.Xp,3,solution.T(:,1)),'rx' )
% %     hold on
% %     plot(solution.T(:,1),speval(solution.Xp,9,solution.T(:,1)),'gx' )
% %     hold on
% %     xlim([0 solution.tf])
% %     ylim([-1 1])
% %     xlabel('Time [s]')
% %     ylabel('Y Position [m]')
% %     legend('Y_1','Y_{com}')
% %     grid on
% %     
% %     subplot(4,2,4)
% %     plot(solution.T(:,1),speval(solution.Xp,4,solution.T(:,1)),'mx' )
% %     hold on
% %     plot(solution.T(:,1),speval(solution.Xp,10,solution.T(:,1)),'bx' )
% %     xlim([0 solution.tf])
% %     ylim([-1 1])
% %     xlabel('Time [s]')
% %     ylabel('Y Velocity [m/s]')
% %     legend('Y_1','Y_{com}')
% %     grid on
% %     
% %     subplot(4,2,5)
% %     plot(solution.T(:,1),speval(solution.Xp,5,solution.T(:,1)),'rx' )
% %     hold on
% %     plot(solution.T(:,1),speval(solution.Xp,11,solution.T(:,1)),'gx' )
% %     hold on
% %     xlim([0 solution.tf])
% %     xlabel('Time [s]')
% %     ylabel('Z Position [m]')
% %     legend('Z_1','Z_{com}')
% %     grid on
% %     
% %     subplot(4,2,6)
% %     plot(solution.T(:,1),speval(solution.Xp,6,solution.T(:,1)),'mx' )
% %     hold on
% %     plot(solution.T(:,1),speval(solution.Xp,12,solution.T(:,1)),'bx' )
% %     xlim([0 solution.tf])
% %     ylim([-1 1])
% %     xlabel('Time [s]')
% %     ylabel('Z Velocity [m/s]')
% %     legend('Z_1','Z_{com}')
% %     grid on
% %     
% %     subplot(4,2,7)
% %     plot(solution.T(:,1),(speval(solution.Xp,1,solution.T(:,1))-speval(solution.Xp,7,solution.T(:,1)))./(speval(solution.Xp,5,solution.T(:,1))-speval(solution.Xp,11,solution.T(:,1))),'rx' )
% %     hold on
% %     plot(solution.T(:,1),(speval(solution.Xp,3,solution.T(:,1))-speval(solution.Xp,9,solution.T(:,1)))./(speval(solution.Xp,5,solution.T(:,1))-speval(solution.Xp,11,solution.T(:,1))),'gx' )
% %     xlim([0 solution.tf])
% %     xlabel('Time [s]')
% %     ylabel('States')
% %     legend('tan(\alpha)','tan(\beta)')
% %     grid on
% %     
% %     subplot(4,2,8)
% %     plot(solution.T(:,1),speval(solution.Up,4,solution.T(:,1)),'mx' )
% %     xlim([0 solution.tf])
% %     xlabel('Time [s]')
% %     ylabel('Cable Tension [N]')
% %     grid on
% %     
% %     figure('Name','Open Loop Inputs')
% %     subplot(3,1,1)
% %     plot(solution.T(:,1),speval(solution.Up,1,solution.T(:,1)),'rx' )
% %     xlim([0 solution.tf])
% %     xlabel('Time [s]')
% %     ylabel('U_{X1} [N]')
% %     grid on
% %     subplot(3,1,2)
% %     plot(solution.T(:,1),speval(solution.Up,2,solution.T(:,1)),'gx' )
% %     xlim([0 solution.tf])
% %     ylim([-1 1])
% %     xlabel('Time [s]')
% %     ylabel('U_{Y1} [N]')
% %     grid on
% %     subplot(3,1,3)
% %     plot(solution.T(:,1),speval(solution.Up,3,solution.T(:,1)),'bx' )
% %     xlim([0 solution.tf])
% %     ylim([26 28])
% %     xlabel('Time [s]')
% %     ylabel('U_{Z1} [N]')
% %     grid on
% % 
% % end