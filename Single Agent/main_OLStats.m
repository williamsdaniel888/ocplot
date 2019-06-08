% mainOLStats - Solves open-loop problem, provides bulk simulation statistics
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

clc;
clear all;
close all;
format compact;
addpath(genpath('..\ICLOCS\src'))
addpath(genpath('..\Ipopt'))
global sol;  
sol=[];                             % Initialize solution structure
N = [5,10,15,20,25,40,50,60,80];
comp_time = zeros(3,9);
max_error = zeros(3,9);
max_rel_error = zeros(3,9);
max_cst_error = zeros(3,9);

for i = 1:length(N)
    options= settings_euler(N(i));            % Get options and solver settings 
    [problem,guess]= SingleAgentQUAV;%    % Fetch the problem definition
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
    [problem,guess]=SingleAgentQUAV;%SingleAgentQUAVProblem;%    % Fetch the problem definition
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
    [problem,guess]=SingleAgentQUAV;%SingleAgentQUAVProblem;%    % Fetch the problem definition
    % 
    tic
    [infoNLP,data,options]=transcribeOCP(problem,guess,options); % Format for NLP solver
    [solution_HS,status,data] = solveNLP(infoNLP,data);      % Solve the NLP
    [solution_HS]=output(problem,solution_HS,options,data,0);         % Output solutions
    comp_time(3,i)=toc;
    max_error(3,i) = max(max(solution_HS.Error));
    max_rel_error(3,i) = max(max(solution_HS.ErrorRelative));
    max_cst_error(3,i) = max(max(solution_HS.ConstraintError));
    
    %options= settings_hscubconst(N(i));            % Get options and solver settings 
    %[problem,guess]=SingleAgentQUAV;%SingleAgentQUAVProblem;%    % Fetch the problem definition
    % 
    %tic
    %[infoNLP,data,options]=transcribeOCP(problem,guess,options); % Format for NLP solver
    %[solution_HSCUBCONST,status,data] = solveNLP(infoNLP,data);      % Solve the NLP
    %[solution_HSCUBCONST]=output(problem,solution_HSCUBCONST,options,data,0);         % Output solutions
    %comp_time(4,i)=toc;
    %max_error(4,i) = max(max(solution_HSCUBCONST.Error));
    %max_rel_error(4,i) = max(max(solution_HSCUBCONST.ErrorRelative));
    %max_cst_error(4,i) = max(max(solution_HSCUBCONST.ConstraintError));

    %sol=[];                             % Initialize solution structure
end

figure('Name','Maximum Error')
plot(log10(N),log10(max_error(1,:)),'rx')
hold on
plot(log10(N),log10(max_error(2,:)),'gx')
hold on
plot(log10(N),log10(max_error(3,:)),'bx')
%hold on
%plot(log10(N),log10(max_error(4,:)),'mx')
xlabel('log_{10} Mesh Points')
ylabel('log_{10} Maximum Error')
legend('Euler','Trapezoidal','HS')
figure('Name','Maximum Relative Error')
plot(log10(N),log10(max_rel_error(1,:)),'rx')
hold on
plot(log10(N),log10(max_rel_error(2,:)),'gx')
hold on
plot(log10(N),log10(max_rel_error(3,:)),'bx')
%hold on
%plot(log10(N),log10(max_rel_error(4,:)),'mx')
xlabel('log_{10} Mesh Points')
ylabel('log_{10} Maximum Relative Error')
legend('Euler','Trapezoidal','HS')
figure('Name','Computation Time')
plot(log10(N),log10(comp_time(1,:)),'rx')
hold on
plot(log10(N),log10(comp_time(2,:)),'gx')
hold on
plot(log10(N),log10(comp_time(3,:)),'bx')
%hold on
%plot(log10(N),log10(comp_time(4,:)),'mx')
xlabel('log_{10} Mesh Points')
ylabel('log_{10} Computation Time [s]')
legend('Euler','Trapezoidal','HS')
figure('Name','Computation Time vs Maximum Error Tradeoff')
plot(log10(comp_time(1,:)),log10(max_error(1,:)),'rx')
hold on
plot(log10(comp_time(2,:)),log10(max_error(2,:)),'gx')
hold on
plot(log10(comp_time(3,:)),log10(max_error(3,:)),'bx')
%hold on
%plot(log10(comp_time(4,:)),log10(max_error(4,:)),'mx')
xlabel('log_{10} Computation Time [s]')
ylabel('log_{10} Maximum Error')
legend('Euler','Trapezoidal','HS')
figure('Name','Computation Time vs Maximum Relative Error Tradeoff')
plot(log10(comp_time(1,:)),log10(max_rel_error(1,:)),'rx')
hold on
plot(log10(comp_time(2,:)),log10(max_rel_error(2,:)),'gx')
hold on
plot(log10(comp_time(3,:)),log10(max_rel_error(3,:)),'bx')
%hold on
%plot(log10(comp_time(4,:)),log10(max_rel_error(4,:)),'mx')
xlabel('log_{10} Computation Time [s]')
ylabel('log_{10} Maximum Relative Error')
legend('Euler','Trapezoidal','HS')