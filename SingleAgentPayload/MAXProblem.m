function [problem,guess] = MAXProblem
% MAXProblem - Script for open-loop simulation using ICLOCS2
%
% Outputs:
%    problem - Structure with information on the optimal control problem
%    guess   - Guess for state, control and multipliers.
%
% Other m-files required: none
% MAT-files required: none
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

%------------- BEGIN CODE --------------
% Plant model name, used for Adigator
problem.data.plantmodel = 'MAXPlant';

 
% Initial time. t0<tf
problem.time.t0=0;

% Final time. Let tf_min=tf_max if tf is fixed.
problem.time.tf_min=5;     
problem.time.tf_max=5; 
guess.tf=5;

% Plant parameters
m_ag = 0.81;
m_pl = 2;
gc = 9.81;
% Parameters bounds. pl=< p <=pu
problem.parameters.pl=[];
problem.parameters.pu=[];
guess.parameters=[];

% Initial conditions for system
problem.states.x0=[2.1 0 0 0 2.2 0 2.1 0 0 0 1 0 ];

% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
problem.states.x0l=[2.1 0 0 0 2.2 0 2.1 0 0 0 1 0 ];
problem.states.x0u=[2.1 0 0 0 2.2 0 2.1 0 0 0 1 0 ];

% State bounds. xl=< x <=xu
problem.states.xl=[2.1 -20 0 0 0 -20 2.1 -20 0 0 0 -20 ];
problem.states.xu=[5.1 +20 0 0 5 20 5.1 +20 0 0 5 20 ];

% State error bounds
problem.states.xErrorTol=(1e-1)*ones(1,12);

% State constraint error bounds
problem.states.xConstraintTol=(1e-1)*ones(1,12);

% Terminal state bounds. xfl=< xf <=xfu
problem.states.xfl=[5.1 0 0 0 2.2 0 5.1 0 0 0 1 0 ];
problem.states.xfu=[5.1 0 0 0 2.2 0 5.1 0 0 0 1 0 ];

% Guess the state trajectories with [x0 xf]
guess.states(:,1)=[2.1 5.1]; %agent 1 x
guess.states(:,2)=[0 0];
guess.states(:,3)=[0 0]; %agent 1 y
guess.states(:,4)=[0 0];
guess.states(:,5)=[2.2 2.2]; %agent 1 z
guess.states(:,6)=[0 0];
guess.states(:,7)=[2.1 5.1]; %payload x
guess.states(:,8)=[0 0];
guess.states(:,9)=[0 0]; %payload y
guess.states(:,10)=[0 0];
guess.states(:,11)=[1 1]; %payload z
guess.states(:,12)=[0 0];

% Number of control actions N 
% Set problem.inputs.N=0 if N is equal to the number of integration steps.  
% Note that the number of integration steps defined in settings.m has to be divisible 
% by the  number of control actions N whenever it is not zero.

problem.inputs.N=0;
% Bounds on first control action
cc = (m_ag+m_pl)*gc;
problem.inputs.u0l=[-100 -100 -100 m_pl*gc];
problem.inputs.u0u=[100 100 100 m_pl*gc];

% Input bounds
problem.inputs.ul=[-100 -100 -100 m_pl*gc-1];
problem.inputs.uu=[100 100 100 m_pl*gc+1];


% Input constraint error bounds
problem.inputs.uConstraintTol= 0.1*ones(1,4);

% Guess the input sequences with [u0 uf]
guess.inputs(:,1)=[20 -20]; %u_x1
guess.inputs(:,2)=[0 0]; %u_y1
guess.inputs(:,3)=[cc cc]; %u_z1
guess.inputs(:,4)=[m_pl*gc m_pl*gc]; %F_1

% Choose the set-points if required
problem.setpoints.states=[];
problem.setpoints.inputs=[];

% Bounds for path constraint function gl =< g(x,u,p,t) =< gu
problem.constraints.gl=[0, 0.5];
problem.constraints.gu=[0, inf];
problem.constraints.gTol=1e-3*ones(1,2);

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

u_x1=u(:,1);
u_y1=u(:,2);
u_z1=u(:,3);

stageCost = u_x1.*u_x1+u_y1.*u_y1+u_z1.*u_z1;

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
m_pl = 2;
g = 9.81;

u_x1=u(:,1);
u_y1=u(:,2);
u_z1=u(:,3);
F_1 = u(:,4);

x1 = x(:,1);
y1 = x(:,3);
z1 = x(:,5);
xc = x(:,7);
yc = x(:,9);
zc = x(:,11);
x1dot = x(:,2);
y1dot = x(:,4);
z1dot = x(:,6);
xcomdot = x(:,8);
ycomdot = x(:,10);
zcomdot = x(:,12);
ta_1 = (x1-xc)./(z1-zc);
tb_1 = (y1-yc)./(z1-zc);

dx(:,1) =  x1dot; %x1.
dx(:,2) =  (1/m_ag)*(u_x1 -F_1.*ta_1); %x1..
dx(:,3) =  y1dot; %x(:,3).
dx(:,4) =  (1/m_ag)*(u_y1 -F_1.*tb_1); %y1..
dx(:,5) =  z1dot; %z1.
dx(:,6) =  (1/m_ag)*(u_z1 -m_ag*g -F_1); %z1..
dx(:,7) =  xcomdot;%xcom.
dx(:,8) =  (1/m_pl)*(F_1.*ta_1);%xcom..
dx(:,9) =  ycomdot;%ycom.
dx(:,10) = (1/m_pl)*(F_1.*tb_1);%ycom..
dx(:,11) = zcomdot;%zcom.
dx(:,12) = (1/m_pl)*(F_1 - m_pl.*g);%zcom..

% dx(:,13) = ((x(:,5)-x(:,11)).*(x(:,2)-x(:,8))-(x(:,1)-x(:,7)).*(x(:,6)-x(:,12)))./((x(:,5)-x(:,11)).^2);%ta1.
% dx(:,14) = ((x(:,5)-x(:,11)).*(x(:,4)-x(:,10))-(x(:,3)-x(:,9)).*(x(:,6)-x(:,12)))./((x(:,5)-x(:,11)).^2);%tb1.
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
x1 = x(:,1);
y1 = x(:,3);
z1 = x(:,5);
xc = x(:,7);
yc = x(:,9);
zc = x(:,11);
sep = sqrt((x1-xc).^2+(y1-yc).^2+(z1-zc).^2);
obst_height = 1;
obst_start = 3.1;
obst_end = 4.1;
clearance = zc-obst_height*(tanh(xc-obst_start)-tanh(xc-obst_end));
c=[sep-1.2, clearance];
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