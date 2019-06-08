% main_OL - Solves open-loop problem
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

clear all;close all;format compact;
global sol;  
sol=[];                             % Initialize solution structure

options= settings_hscubconst(10);                  % Get options and solver settings 
[problem,guess]=SingleAgentQUAV;          % Fetch the problem definition
errorHistory=zeros(2,length(problem.states.x0));
npsegmentHistory=zeros(2,1);
ConstraintErrorHistory=zeros(2,length(problem.constraintErrorTol));
timeHistory=zeros(1,2);
iterHistory=zeros(1,2);
solutionHistory=cell(1,2);

maxAbsError=1e9;
i=1; imax=3; %imax default: 50

while (any(maxAbsError>problem.states.xErrorTol) || any(maxAbsConstraintError>problem.constraintErrorTol)) && i<=imax    
    [infoNLP,data,options]=transcribeOCP(problem,guess,options); % Format for NLP solver
    [solution,status,data] = solveNLP(infoNLP,data);      % Solve the NLP
    [solution]=output(problem,solution,options,data,4);         % Output solutions

    
    maxAbsError=max(abs(solution.Error));
    maxAbsConstraintError=max(solution.ConstraintError);
    errorHistory(i,:)=maxAbsError;
    iterHistory(i)=status.iter;
    ConstraintErrorHistory(i,:)=maxAbsConstraintError;
    timeHistory(i)=solution.computation_time;
    solutionHistory{i}=solution;
    
    if (any(maxAbsError>problem.states.xErrorTol) || any(maxAbsConstraintError>problem.constraintErrorTol)) && i<=imax
        [ options, guess ] = doMeshRefinement( options, problem, guess, data, solution, i );
    end
    i=i+1;

end

MeshRefinementHistory.errorHistory=errorHistory;
MeshRefinementHistory.timeHistory=timeHistory;
MeshRefinementHistory.iterHistory=iterHistory;
MeshRefinementHistory.ConstraintErrorHistory=ConstraintErrorHistory;


%%
%%
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
    legend('X Position [m]','X Velocity [m.s^{-1}]')
    grid on
    subplot(3,1,2)
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,3,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'go' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,4,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'yd' )
    plot(xx,speval(solution.Xp,3,solution.TSeg_Bar,xx),'g-' )
    plot(xx,speval(solution.Xp,4,solution.TSeg_Bar,xx),'y-' )
    xlabel('Time [s]')
    ylabel('States')
    legend('Y Position [m]','Y Velocity [m.s^{-1}]')
    grid on
    subplot(3,1,3)
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,5,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'bo' )
    hold on
    plot([solution.T(:,1); solution.tf],speval(solution.Xp,6,solution.TSeg_Bar,[solution.T(:,1); solution.tf]),'cd' )
    plot(xx,speval(solution.Xp,3,solution.TSeg_Bar,xx),'b-' )
    plot(xx,speval(solution.Xp,4,solution.TSeg_Bar,xx),'c-' )
    xlabel('Time [s]')
    ylabel('States')
    legend('Z Position [m]','Z Velocity [m.s^{-1}]')
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
    subplot(3,2,1)
    plot(solution.T(:,1),speval(solution.Xp,1,solution.T(:,1)),'rx' )
    xlim([0 solution.tf])
    ylim([0 6])
    xlabel('Time [s]')
    ylabel('X Position [m]')
    grid on
    subplot(3,2,2)
    plot(solution.T(:,1),speval(solution.Xp,2,solution.T(:,1)),'rx' )
    xlim([0 solution.tf])
    ylim([0 1])
    xlabel('Time [s]')
    ylabel('X Velocity [m.s^{-1}]')
    grid on
    subplot(3,2,3)
    plot(solution.T(:,1),speval(solution.Xp,3,solution.T(:,1)),'gx' )
    xlim([0 solution.tf])
    ylim([-1 1])
    xlabel('Time [s]')
    ylabel('Y Position [m]')
    grid on
    subplot(3,2,4)
    plot(solution.T(:,1),speval(solution.Xp,4,solution.T(:,1)),'gx' )
    xlim([0 solution.tf])
    ylim([-1 1])
    xlabel('Time [s]')
    ylabel('Y Velocity [m.s^{-1}]')
    grid on
    subplot(3,2,5)
    plot(solution.T(:,1),speval(solution.Xp,5,solution.T(:,1)),'bx' )
    hold on
    spline = speval(solution.Xp,1,xx);
    obstacle_time_start = xx(find(spline>3.1,1));
    obstacle_time_end = xx(find(spline>4.1,1));
    obs_height=1.5;
    k=1;
    plot(xx,obs_height.*(tanh(k*(xx-obstacle_time_start)) - tanh(k*(xx-obstacle_time_end))))
    xlim([0 solution.tf])
    ylim([0 2.7])
    xlabel('Time [s]')
    ylabel('Z Position [m]')
    legend('Z Position [m]','Obstacle Height [m]')
    grid on
    subplot(3,2,6)
    plot(solution.T(:,1),speval(solution.Xp,6,solution.T(:,1)),'bx' )
    xlim([0 solution.tf])
    ylim([-1 1])
    xlabel('Time [s]')
    ylabel('Z Velocity [m.s^{-1}]')
    grid on
    
    figure
    subplot(3,1,1)
    plot(solution.T(:,1),speval(solution.Up,1,solution.T),'rx' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    grid on
    ylabel('u_X [N]')
    subplot(3,1,2)
    plot(solution.T(:,1),speval(solution.Up,2,solution.T),'gx' )
    xlim([0 solution.tf])
    ylim([-1 1])
    xlabel('Time [s]')
    grid on
    ylabel('u_Y [N]')
    subplot(3,1,3)
    plot(solution.T(:,1),speval(solution.Up,3,solution.T),'bx' )
    xlim([0 solution.tf])
    xlabel('Time [s]')
    grid on
    ylabel('u_Z [N]')
end