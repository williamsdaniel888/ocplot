function [problem,guess] = MultiAgentQUAVProblem
%SingleAgentQUAV Control (Double Integrator Minimum Actuator Effort Squared Repositioning) Problem
%
% The problem was adapted from Example 4.11 from
% J. Betts, "Practical Methods for Optimal Control and Estimation Using Nonlinear Programming: Second Edition," Advances in Design and Control, Society for Industrial and Applied Mathematics, 2010.
%
% Outputs:
%    problem - Structure with information on the optimal control problem
%    guess   - Guess for state, control and multipliers.
%
% Other m-files required: none
% MAT-files required: none
%
% Copyright (C) 2018 Yuanbo Nie, Omar Faqir, and Eric Kerrigan. All Rights Reserved.
% The contribution of Paola Falugi, Eric Kerrigan and Eugene van Wyk for the work on ICLOCS Version 1 (2010) is kindly acknowledged.
% This code is published under the BSD License.
% Department of Aeronautics and Department of Electrical and Electronic Engineering,
% Imperial College London London  England, UK 
% ICLOCS (Imperial College London Optimal Control) Version 2.0 
% 1 May 2018
% iclocs@imperial.ac.uk

%------------- BEGIN CODE --------------
% Plant model name, used for Adigator
problem.data.plantmodel = 'MultiAgentQUAVPlant';

 
% Initial time. t0<tf
problem.time.t0=0;

% Final time. Let tf_min=tf_max if tf is fixed.
problem.time.tf_min=30;     
problem.time.tf_max=30; 
guess.tf=30;

% Parameters bounds. pl=< p <=pu
problem.parameters.pl=[];
problem.parameters.pu=[];
guess.parameters=[];

% Initial conditions for system
problem.states.x0=[2.1 0 0.4 0 2.2 0 3.2 0 -0.3 0 2.2 0 3.8 0 0.2 0 2.2 0];

% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
problem.states.x0l=[2.1 0 0.4 0 2.2 0 3.2 0 -0.3 0 2.2 0 3.8 0 0.2 0 2.2 0];
problem.states.x0u=[2.1 0 0.4 0 2.2 0 3.2 0 -0.3 0 2.2 0 3.8 0 0.2 0 2.2 0];



% State bounds. xl=< x <=xu
problem.states.xl=[0 -10 0.4 -10 2.2 -10 0 -10 -0.3 -10 2.2 -10 0 -10 0.2 -10 2.2 -10];
problem.states.xu=[10 10 0.4 10 2.2 10 10 10 -0.3 10 2.2 10 10 10 0.2 10 2.2 10];


% State error bounds
problem.states.xErrorTol=[1e-2 1e-2 1e-2 1e-2 1e-2 1e-2 1e-2 1e-2 1e-2 1e-2 1e-2 1e-2 1e-2 1e-2 1e-2 1e-2 1e-2 1e-2];


% State constraint error bounds
problem.states.xConstraintTol=[1e-2 1e-2 1e-2 1e-2 1e-2 1e-2 1e-2 1e-2 1e-2 1e-2 1e-2 1e-2 1e-2 1e-2 1e-2 1e-2 1e-2 1e-2];


% Terminal state bounds. xfl=< xf <=xfu
problem.states.xfl=[5.1 0 0.4 0 2.2 0 6.2 0 -0.3 0 2.2 0 6.8 0 0.2 0 2.2 0];
problem.states.xfu=[5.1 0 0.4 0 2.2 0 6.2 0 -0.3 0 2.2 0 6.8 0 0.2 0 2.2 0];

% Guess the state trajectories with [x0 xf]
guess.states(:,1)=[2.1 5.1];
guess.states(:,2)=[0 0];
guess.states(:,3)=[0.4 0.4];
guess.states(:,4)=[0 0];
guess.states(:,5)=[2.2 2.2];
guess.states(:,6)=[0 0];
guess.states(:,7)=[3.2 6.2];
guess.states(:,8)=[0 0];
guess.states(:,9)=[-0.3 -0.3];
guess.states(:,10)=[0 0];
guess.states(:,11)=[2.2 2.2];
guess.states(:,12)=[0 0];
guess.states(:,13)=[3.8 6.8];
guess.states(:,14)=[0 0];
guess.states(:,15)=[0.2 0.2];
guess.states(:,16)=[0 0];
guess.states(:,17)=[2.2 2.2];
guess.states(:,18)=[0 0];

% Number of control actions N 
% Set problem.inputs.N=0 if N is equal to the number of integration steps.  
% Note that the number of integration steps defined in settings.m has to be divisible 
% by the  number of control actions N whenever it is not zero.

problem.inputs.N=0;

% Input bounds
problem.inputs.ul=[-5 -5 -10 -5 -5 -10 -5 -5 -10];
problem.inputs.uu=[10 10 30 10 10 30 10 10 30];

% Bounds on first control action
problem.inputs.u0l=[-10 -10 0 -10 -10 0 -10 -10 0];
problem.inputs.u0u=[10 10 20 10 10 20 10 10 20]; 

% Input constraint error bounds
problem.inputs.uConstraintTol=[0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1];

% Guess the input sequences with [u0 uf]
guess.inputs(:,1)=[0 0];
guess.inputs(:,2)=[0 0];
guess.inputs(:,3)=[9.81 9.81];
guess.inputs(:,4)=[0 0];
guess.inputs(:,5)=[0 0];
guess.inputs(:,6)=[9.81 9.81];
guess.inputs(:,7)=[0 0];
guess.inputs(:,8)=[0 0];
guess.inputs(:,9)=[9.81 9.81];

% Choose the set-points if required
problem.setpoints.states=[];
problem.setpoints.inputs=[];

% Bounds for path constraint function gl =< g(x,u,p,t) =< gu
problem.constraints.gl=[];%1;
problem.constraints.gu=[];%inf;
problem.constraints.gTol=[];%[1e-6];

% Bounds for boundary constraints bl =< b(x0,xf,u0,uf,p,t0,tf) =< bu
problem.constraints.bl=[];
problem.constraints.bu=[];


% store the necessary problem parameters used in the functions
% problem.data = []; % [lb]

% problem.data.tau = tau;
% Get function handles and return to Main.m
problem.functions={@L,@E,@f,@g,@avrc,@b};
problem.functions_unscaled={@L_unscaled,@E_unscaled,@f_unscaled,@g_unscaled,@avrc_unscaled,@b_unscaled};
problem.constraintErrorTol=[problem.constraints.gTol,problem.constraints.gTol,problem.states.xConstraintTol,problem.states.xConstraintTol,problem.inputs.uConstraintTol,problem.inputs.uConstraintTol];

%------------- END OF CODE --------------

function stageCost=L_unscaled(x,xr,u,ur,p,t,vdat)

% L_unscaled - Returns the stage cost.
% The function must be vectorized and
% xi, ui are column vectors taken as x(:,i) and u(:,i) (i denotes the i-th
% variable)
% 
% Syntax:  stageCost = L(x,xr,u,ur,p,t,data)
%
% Inputs:
%    x  - state vector
%    xr - state reference
%    u  - input
%    ur - input reference
%    p  - parameter
%    t  - time
%    data- structured variable containing the values of additional data used inside
%          the function
%
% Output:
%    stageCost - Scalar or vectorized stage cost
%
%  Remark: If the stagecost does not depend on variables it is necessary to multiply
%          the assigned value by t in order to have right vector dimesion when called for the optimization. 
%          Example: stageCost = 0*t;

%------------- BEGIN CODE --------------
u1 = u(:,1);
u2 = u(:,2);
u3 = u(:,3);
u4 = u(:,4);
u5 = u(:,5);
u6 = u(:,6);
u7 = u(:,7);
u8 = u(:,8);
u9 = u(:,9);
stageCost = u1.*u1+u2.*u2+u3.*u3+u4.*u4+u5.*u5+u6.*u6+u7.*u7+u8.*u8+u9.*u9;

%------------- END OF CODE --------------


function boundaryCost=E_unscaled(x0,xf,u0,uf,p,t0,tf,vdat) 

% E_unscaled - Returns the boundary value cost
%
% Syntax:  boundaryCost=E(x0,xf,u0,uf,p,tf,data)
%
% Inputs:
%    x0  - state at t=0
%    xf  - state at t=tf
%    u0  - input at t=0
%    uf  - input at t=tf
%    p   - parameter
%    tf  - final time
%    data- structured variable containing the values of additional data used inside
%          the function
%
% Output:
%    boundaryCost - Scalar boundary cost
%
%------------- BEGIN CODE --------------

boundaryCost=0;

%------------- END OF CODE --------------


function dx = f_unscaled(x,u,p,t,vdat)
% f_unscaled - Returns the ODE right hand side where x'= f(x,u,p,t)
% The function must be vectorized and
% xi, ui, pi are column vectors taken as x(:,i), u(:,i) and p(:,i). Each
% state corresponds to one column of dx.
% 
% 
% Syntax:  dx = f(x,u,p,t,data)
%
% Inputs:
%    x  - state vector
%    u  - input
%    p  - parameter
%    t  - time
%    data-structured variable containing the values of additional data used inside
%          the function 
%
% Output:
%    dx - time derivative of x
%
%  Remark: If the i-th ODE right hand side does not depend on variables it is necessary to multiply
%          the assigned value by a vector of ones with the same length  of t  in order 
%          to have  a vector with the right dimesion  when called for the optimization. 
%          Example: dx(:,i)= 0*ones(size(t,1)); 
%
%------------- BEGIN CODE --------------
m_ag = 0.81;
% m_pl = 20;
g = 9.81;
u_x1=u(:,1);
u_y1=u(:,2);
u_z1=u(:,3);
u_x2=u(:,4);
u_y2=u(:,5);
u_z2=u(:,6);
u_x3=u(:,7);
u_y3=u(:,8);
u_z3=u(:,9);
a_1=p(:,1);
b_1=p(:,2);
a_2=p(:,3);
b_2=p(:,4);
a_3=p(:,5);
b_3=p(:,6);
F_1=p(:,7);
F_2=p(:,8);
F_3=p(:,9);

x2 = x(:,2);
x4 = x(:,4);
x6 = x(:,6);
x8 = x(:,8);
x10 = x(:,10);
x12 = x(:,12);
x14 = x(:,14);
x16 = x(:,16);
x18 = x(:,18);

dx(:,1) = x2; %x1.
dx(:,2) = (1/m_ag)*(u_x1-F_1*tan(a_1)); %x1..
dx(:,3) = x4; %y1.
dx(:,4) = (1/m_ag)*(u_y1 - F_1*tan(b_1)); %y1..
dx(:,5) = x6; %z1.
dx(:,6) = (1/m_ag)*(u_z1 - F_1 - m_ag*g); %z1..
dx(:,7) = x8; %x2.
dx(:,8) = (1/m_ag)*(u_x2 - F_1*tan(a_2)); %x2..
dx(:,9) = x10; %y2.
dx(:,10) = (1/m_ag)*(u_y2 - F_1*tan(b_2)); %y2..
dx(:,11) = x12; %z2.
dx(:,12) = (1/m_ag)*(u_z2 - F_2 - m_ag*g); %z2..
dx(:,13) = x14; %x3.
dx(:,14) = (1/m_ag)*(u_x3 - F_1*tan(a_3)); %x3..
dx(:,15) = x16; %y3.
dx(:,16) = (1/m_ag)*(u_y3 - F_1*tan(b_3)); %y3..
dx(:,17) = x18; %z3.
dx(:,18) = (1/m_ag)*(u_z3 - F_3 - m_ag*g); %z3..

%------------- END OF CODE --------------


function c=g_unscaled(x,u,p,t,vdat)

% g_unscaled - Returns the path constraint function where gl =< g(x,u,p,t) =< gu
% The function must be vectorized and
% xi, ui, pi are column vectors taken as x(:,i), u(:,i) and p(:,i). Each
% constraint corresponds to one column of c
% 
% Syntax:  c=g(x,u,p,t,data)
%
% Inputs:
%    x  - state vector
%    u  - input
%    p  - parameter
%    t  - time
%   data- structured variable containing the values of additional data used inside
%          the function
%
% Output:
%    c - constraint function
%
%------------- BEGIN CODE --------------
% x1=x(:,1);
% x5=x(:,5); %z
% 
% k=1;
% obs_height = 7;
% obs_width_start = 9;
% obs_width_end = 12;
% % c=[x5+(2/90).*x1.*x1-(2/3).*x1];
% %c=[x5-(x1.*(x1-30)+4)]; 
% c=[x5-obs_height*0.5*(tanh(k*(x1-obs_width_start)) - tanh(k*(x1-obs_width_end)))];
c=[];
%------------- END OF CODE --------------

function bc=b_unscaled(x0,xf,u0,uf,p,t0,tf,vdat,varargin)

% b_unscaled - Returns a column vector containing the evaluation of the boundary constraints: bl =< bf(x0,xf,u0,uf,p,t0,tf) =< bu
%
% Syntax:  bc=b(x0,xf,u0,uf,p,tf,data)
%
% Inputs:
%    x0  - state at t=0
%    xf  - state at t=tf
%    u0  - input at t=0
%    uf  - input at t=tf
%    p   - parameter
%    tf  - final time
%    data- structured variable containing the values of additional data used inside
%          the function
%
%          
% Output:
%    bc - column vector containing the evaluation of the boundary function 
%
%------------- BEGIN CODE --------------
varargin=varargin{1};
bc=[];
%------------- END OF CODE --------------
% When adpative time interval add constraint on time
%------------- BEGIN CODE --------------
if length(varargin)==2
    options=varargin{1};
    t_segment=varargin{2};
    if ((strcmp(options.transcription,'hpLGR')) || (strcmp(options.transcription,'globalLGR')))  && options.adaptseg==1 
        if size(t_segment,1)>size(t_segment,2)
            bc=[bc;diff(t_segment)];
        else
            bc=[bc;diff(t_segment)'];
        end
    end
end

%------------- END OF CODE --------------

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Leave the following unchanged! %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function stageCost=L(x,xr,u,ur,p,t,vdat)

% L - Returns the stage cost.
% Warp function
%------------- BEGIN CODE --------------

if isfield(vdat,'Xscale')
    x=scale_variables_back( x, vdat.Xscale, vdat.Xshift );
    if ~isempty(xr)
        xr=scale_variables_back( xr, vdat.Xscale, vdat.Xshift );
    end
    u=scale_variables_back( u, vdat.Uscale, vdat.Ushift );
    if isfield(vdat,'Pscale')
        p=scale_variables_back( p, vdat.Pscale, vdat.Pshift );
    end
    if ~isempty(ur)
        ur=scale_variables_back( ur, vdat.Uscale, vdat.Ushift );
    end
    if strcmp(vdat.mode.currentMode,'Feasibility')
        stageCost=0*t;
    else
        stageCost=L_unscaled(x,xr,u,ur,p,t,vdat);
    end
else
    if strcmp(vdat.mode.currentMode,'Feasibility')
        stageCost=0*t;
    else
        stageCost=L_unscaled(x,xr,u,ur,p,t,vdat);
    end
end

%------------- END OF CODE --------------


function boundaryCost=E(x0,xf,u0,uf,p,t0,tf,vdat) 

% E - Returns the boundary value cost
% Warp function
%------------- BEGIN CODE --------------
if isfield(vdat,'Xscale')
    x0=scale_variables_back( x0', vdat.Xscale, vdat.Xshift );
    xf=scale_variables_back( xf', vdat.Xscale, vdat.Xshift );
    u0=scale_variables_back( u0', vdat.Uscale, vdat.Ushift );
    uf=scale_variables_back( uf', vdat.Uscale, vdat.Ushift );
    if isfield(vdat,'Pscale')
        p=scale_variables_back( p', vdat.Pscale, vdat.Pshift );
    end
    if strcmp(vdat.mode.currentMode,'Feasibility')
        boundaryCost=sum(sum(p(:,end-vdat.mode.np*2+1:end)));
    else
        boundaryCost=E_unscaled(x0,xf,u0,uf,p,t0,tf,vdat);
    end
else
    if strcmp(vdat.mode.currentMode,'Feasibility')
        boundaryCost=sum(sum(p(:,end-vdat.mode.np*2+1:end)));
    else
        boundaryCost=E_unscaled(x0,xf,u0,uf,p,t0,tf,vdat);
    end
end


%------------- END OF CODE --------------


function dx = f(x,u,p,t,vdat)
% f - Returns the ODE right hand side where x'= f(x,u,p,t)
% Warp function
%------------- BEGIN CODE --------------

if isfield(vdat,'Xscale')
    x=scale_variables_back( x, vdat.Xscale, vdat.Xshift );
    u=scale_variables_back( u, vdat.Uscale, vdat.Ushift );
    if isfield(vdat,'Pscale')
        p=scale_variables_back( p, vdat.Pscale, vdat.Pshift );
    end
    dx = f_unscaled(x,u,p,t,vdat);
    dx= scale_variables( dx, vdat.Xscale, 0 );
else
    dx = f_unscaled(x,u,p,t,vdat);
end

%------------- END OF CODE --------------

function c=g(x,u,p,t,vdat)

% g - Returns the path constraint function where gl =< g(x,u,p,t) =< gu
% Warp function
%------------- BEGIN CODE --------------

if isfield(vdat,'Xscale')
    x=scale_variables_back( x, vdat.Xscale, vdat.Xshift );
    u=scale_variables_back( u, vdat.Uscale, vdat.Ushift );
    if isfield(vdat,'Pscale')
        p=scale_variables_back( p, vdat.Pscale, vdat.Pshift );
    end
    c = g_unscaled(x,u,p,t,vdat);
else
    c = g_unscaled(x,u,p,t,vdat);
end

if isfield(vdat,'gFilter')
    c(:,vdat.gFilter)=[];
end

if strcmp(vdat.mode.currentMode,'Feasibility')
    c=[c-p(:,end-vdat.mode.np*2+1:end-vdat.mode.np) c+p(:,end-vdat.mode.np+1:end)];
end

%------------- END OF CODE --------------

function cr=avrc(x,u,p,t,data)

% avrc - Returns the rate constraint algebraic function where [xrl url] =<
% avrc(x,u,p,t) =< [xru uru]
% The function must be vectorized and
% xi, ui, pi are column vectors taken as x(:,i), u(:,i) and p(:,i). Each
% constraint corresponds to one column of c
% 
% Syntax:  cr=avrc(x,u,p,t,data)
%
% Inputs:
%    x  - state vector
%    u  - input
%    p  - parameter
%    t  - time
%   data- structured variable containing the values of additional data used inside
%          the function
%
% Output:
%    cr - constraint function
%
%
%------------- BEGIN CODE --------------
[ cr ] = addRateConstraint( x,u,p,t,data );
%------------- END OF CODE --------------



function bc=b(x0,xf,u0,uf,p,t0,tf,vdat,varargin)
% b - Returns a column vector containing the evaluation of the boundary constraints: bl =< bf(x0,xf,u0,uf,p,t0,tf) =< bu
% Warp function
%------------- BEGIN CODE --------------
bc=b_unscaled(x0,xf,u0,uf,p,t0,tf,vdat,varargin);
if isfield(vdat,'Xscale')
    if ~isempty(bc)
        x0=scale_variables_back( x0', vdat.Xscale, vdat.Xshift );
        xf=scale_variables_back( xf', vdat.Xscale, vdat.Xshift );
        u0=scale_variables_back( u0', vdat.Uscale, vdat.Ushift );
        uf=scale_variables_back( uf', vdat.Uscale, vdat.Ushift );
        if isfield(vdat,'Pscale')
            p=scale_variables_back( p', vdat.Pscale, vdat.Pshift );
        end
        bc=b_unscaled(x0,xf,u0,uf,p,t0,tf,vdat,varargin);
    end
end


%------------- END OF CODE ---------------------