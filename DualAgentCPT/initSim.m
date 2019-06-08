% initSim - Initialization for Closed-loop Simulation with ICLOCS2
%
% Input:
%    tpx0 - vector in format [current time; current state]
%
% Output:
%    tfpu - vector in format [terminal time; computation time; input to implement]
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
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
close all
clear all
addpath(genpath('..\..\ICLOCS\src'))
addpath(genpath('..\..\Ipopt'))
global infoNLP data simtime solution tstep opt_step N opt_t_min
simtime=0; % initialize simulation time
tstep=0.01; % time step of simulation
opt_step=1    ; % time interval for re-optimization, set to zero for update ASAP
opt_t_min=1; % stop re-optimizing if solution.tf<opt_t_min
N=25; % initial number of mesh points

options= settings_hscubconst(N);                  % Get options and solver settings
[problem,guess]=MAGProblem;          % Fetch the problem definition

% initialize problem
solution.tf=problem.time.tf_max; 
solution.status.status=0;
solution.t_ref=0;
par=problem.data;
[infoNLP,data]=transcribeOCP(problem,guess,options); 
[nt,np,n,m,ng,nb,M,N,ns]=deal(data.sizes{1:9});
disp('initsim done')


