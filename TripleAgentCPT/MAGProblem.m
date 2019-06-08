function [problem,guess] = MAGProblem
% MAGProblem - Problem file for ICLOCS2
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

%------------- BEGIN CODE --------------
% Plant model name, used for Adigator
problem.data.plantmodel = 'MAGPlant_B';
 
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
cl = 1.2;
asep = 0.5;
% Parameters bounds. pl=< p <=pu
problem.parameters.pl=[];
problem.parameters.pu=[];
guess.parameters=[];

% Initial conditions for system
problem.states.x0=[2.1 0 0 0 2.2 0, 2.1 0 0 0 1 0, 3 0 0 0 2.2 0, 3 0 0 0 1 0, 3.9 0 0 0 2.2 0, 3.9 0 0 0 1 0];

% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
problem.states.x0l=[2.1 0 0 0 2.2 0, 2.1 0 0 0 1 0, 3 0 0 0 2.2 0, 3 0 0 0 1 0, 3.9 0 0 0 2.2 0, 3.9 0 0 0 1 0];
problem.states.x0u=[2.1 0 0 0 2.2 0, 2.1 0 0 0 1 0, 3 0 0 0 2.2 0, 3 0 0 0 1 0, 3.9 0 0 0 2.2 0, 3.9 0 0 0 1 0];

% State bounds. xl=< x <=xu
problem.states.xl=[2.1 -20 0 0 2.2 -20,  2.1 -20 0 0 1 -20, 3 -20 0 0 2.2 -20,  3 -20 0 0 1 -20, 3.9 -20 0 0 2.2 -20,  3.9 -20 0 0 1 -20, ];
problem.states.xu=[5.1 +20 0 0 5 +20,  5.1 +20 0 0 5 +20, 6 +20 0 0 5 +20,  6 +20 0 0 5 +20, 6.9 +20 0 0 5 +20,  6.9 +20 0 0 5 +20, ];

% State error bounds
problem.states.xErrorTol=(1e-1)*ones(1,36);

% State constraint error bounds
problem.states.xConstraintTol=(1e-1)*ones(1,36);

% Terminal state bounds. xfl=< xf <=xfu
problem.states.xfl=[5.1 0 0 0 2.2 0, 5.1 0 0 0 1 0, 6 0 0 0 2.2 0, 6 0 0 0 1 0, 6.9 0 0 0 2.2 0, 6.9 0 0 0 1 0];
problem.states.xfu=[5.1 0 0 0 2.2 0, 5.1 0 0 0 1 0, 6 0 0 0 2.2 0, 6 0 0 0 1 0, 6.9 0 0 0 2.2 0, 6.9 0 0 0 1 0];

% Guess the state trajectories with [x0 xf]
guess.states(:,1)=[2.1 5.1]; %agent 1
guess.states(:,2)=[0 0];
guess.states(:,3)=[0 0];
guess.states(:,4)=[0 0];
guess.states(:,5)=[2.2 2.2];
guess.states(:,6)=[0 0];
guess.states(:,7)=[2.1 5.1]; %payload
guess.states(:,8)=[0 0];
guess.states(:,9)=[0 0];
guess.states(:,10)=[0 0];
guess.states(:,11)=[1 1]; 
guess.states(:,12)=[0 0]; 
guess.states(:,13)=[3 6]; %agent 2
guess.states(:,14)=[0 0];
guess.states(:,15)=[0 0];
guess.states(:,16)=[0 0];
guess.states(:,17)=[2.2 2.2];
guess.states(:,18)=[0 0];
guess.states(:,19)=[3 6]; %payload
guess.states(:,20)=[0 0];
guess.states(:,21)=[0 0];
guess.states(:,22)=[0 0];
guess.states(:,23)=[1 1]; 
guess.states(:,24)=[0 0]; 
guess.states(:,25)=[3.9 6.9]; %agent 3
guess.states(:,26)=[0 0];
guess.states(:,27)=[0 0];
guess.states(:,28)=[0 0];
guess.states(:,29)=[2.2 2.2];
guess.states(:,30)=[0 0];
guess.states(:,31)=[3.9 6.9]; %payload
guess.states(:,32)=[0 0];
guess.states(:,33)=[0 0];
guess.states(:,34)=[0 0];
guess.states(:,35)=[1 1]; 
guess.states(:,36)=[0 0];

% Number of control actions N 
% Set problem.inputs.N=0 if N is equal to the number of integration steps.  
% Note that the number of integration steps defined in settings.m has to be divisible 
% by the  number of control actions N whenever it is not zero.

problem.inputs.N=0;
% Bounds on first control action
cc = (m_ag+m_pl/3)*gc;
problem.inputs.u0l=[-50 -50 -50  m_pl*gc/3, -50 -50 -50   m_pl*gc/3, -50 -50 -50    m_pl*gc/3];
problem.inputs.u0u=[ 50 50 50  m_pl*gc/3,   50 50 50 m_pl*gc/3,  50 50 50 m_pl*gc/3];

% Input bounds
problem.inputs.ul=[-50 -50 -50  m_pl*gc/3-5, -50 -50 -50   m_pl*gc/3-5, -50 -50 -50    m_pl*gc/3-5];
problem.inputs.uu=[ 50 50 50  m_pl*gc/3+5,   50 50 50 m_pl*gc/3+5,  50 50 50 m_pl*gc/3+5];


% Input constraint error bounds
problem.inputs.uConstraintTol= 0.1*ones(1,12);

% Guess the input sequences with [u0 uf]
guess.inputs(:,1)=[0.5 -0.5]; %u_x1
guess.inputs(:,2)=[0 0];
guess.inputs(:,3)=[cc cc];
guess.inputs(:,4)=[m_pl*gc/3 m_pl*gc/3]; %F_1
guess.inputs(:,5)=[0.5 -0.5]; %u_x2
guess.inputs(:,6)=[0 0];
guess.inputs(:,7)=[cc cc];
guess.inputs(:,8)=[m_pl*gc/3 m_pl*gc/3]; %F_2
guess.inputs(:,9)=[0.5 -0.5]; %u_x3
guess.inputs(:,10)=[0 0];
guess.inputs(:,11)=[cc cc];
guess.inputs(:,12)=[m_pl*gc/3 m_pl*gc/3]; %F_3

% Choose the set-points if required
problem.setpoints.states=[];
problem.setpoints.inputs=[];

% Bounds for path constraint function gl =< g(x,u,p,t) =< gu
problem.constraints.gl=[asep asep asep, 0, 0 0 0, 0.5 0.5 0.5];
problem.constraints.gu=[2*cl 2*cl 3*cl, 0, 0 0 0, inf inf inf];
problem.constraints.gTol=1e-2*ones(1,10);

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
u_x2=u(:,5);
u_y2=u(:,6);
u_z2=u(:,7);
u_x3=u(:,9);
u_y3=u(:,10);
u_z3=u(:,11);

stageCost = u_x1.*u_x1+u_y1.*u_y1+u_z1.*u_z1 +u_x2.*u_x2+u_y2.*u_y2+u_z2.*u_z2 +u_x3.*u_x3+u_y3.*u_y3+u_z3.*u_z3;

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
% global x1g x2g x3g
m_ag = 0.81;
m_pl = 2;
g = 9.81;


u_x1=u(:,1);
u_y1=u(:,2);
u_z1=u(:,3);
F_1 = u(:,4);
u_x2=u(:,5);
u_y2=u(:,6);
u_z2=u(:,7);
F_2 = u(:,8);
u_x3=u(:,9);
u_y3=u(:,10);
u_z3=u(:,11);
F_3 = u(:,12);

x1 = x(:,1);
y1 = x(:,3); 
z1 = x(:,5);
x1dot = x(:,2);
y1dot = x(:,4);
z1dot = x(:,6);
x2 = x(:,13);
y2 = x(:,15); 
z2 = x(:,17);
x2dot = x(:,14);
y2dot = x(:,16);
z2dot = x(:,18);
x3 = x(:,25);
y3 = x(:,27); 
z3 = x(:,29);
x3dot = x(:,26);
y3dot = x(:,28);
z3dot = x(:,30);
% x1g = x1; 
% x2g = x2;
% x3g = x3;

xc1 = x(:,7);
yc1 = x(:,9);
zc1 = x(:,11);
xc1dot = x(:,8);
yc1dot = x(:,10);
zc1dot = x(:,12);
xc2 = x(:,19);
yc2 = x(:,21);
zc2 = x(:,23);
xc2dot = x(:,20);
yc2dot = x(:,22);
zc2dot = x(:,24);
xc3 = x(:,31);
yc3 = x(:,33);
zc3 = x(:,35);
xc3dot = x(:,32);
yc3dot = x(:,34);
zc3dot = x(:,36);

ta_1 = (x1-xc1)./(z1-zc1);
tb_1 = (y1-yc1)./(z1-zc1);
ta_2 = (x2-xc2)./(z2-zc2);
tb_2 = (y2-yc2)./(z2-zc2);
ta_3 = (x3-xc3)./(z3-zc3);
tb_3 = (y3-yc3)./(z3-zc3);

Fxc = (F_1.*ta_1+F_2.*ta_2+F_3.*ta_3);
Fyc = (F_1.*tb_1+F_2.*tb_2+F_3.*tb_3);
Fzc = (F_1+F_2+F_3); 

dx(:,1) =  x1dot; %x1.
dx(:,2) =  (1/m_ag)*(u_x1 -F_1.*ta_1); %x1..
dx(:,3) =  y1dot; %y1.
dx(:,4) =  (1/m_ag)*(u_y1 -F_1.*tb_1); %y1..
dx(:,5) =  z1dot; %z1.
dx(:,6) =  (1/m_ag)*(u_z1 -F_1 -m_ag*g); %z1..

dx(:,7) =  xc1dot;%xcom.
dx(:,8) =  (1/m_pl)*Fxc;%xcom..
dx(:,9) =  yc1dot;%ycom.
dx(:,10) = (1/m_pl)*Fyc;%ycom..
dx(:,11) = zc1dot;%zcom.
dx(:,12) = (1/m_pl)*Fzc-g;%zcom..

dx(:,13) =  x2dot; %x1.
dx(:,14) =  (1/m_ag)*(u_x2 -F_2.*ta_2); %x1..
dx(:,15) =  y2dot; %y1.
dx(:,16) =  (1/m_ag)*(u_y2 -F_2.*tb_2); %y1..
dx(:,17) =  z2dot; %z1.
dx(:,18) =  (1/m_ag)*(u_z2 -F_2 -m_ag*g); %z1..

dx(:,19) =  xc2dot;%xcom.
dx(:,20) =  (1/m_pl)*Fxc;%xcom..
dx(:,21) =  yc2dot;%ycom.
dx(:,22) = (1/m_pl)*Fyc;%ycom..
dx(:,23) = zc2dot;%zcom.
dx(:,24) = (1/m_pl)*Fzc-g;%zcom..

dx(:,25) =  x3dot; %x1.
dx(:,26) =  (1/m_ag)*(u_x3 -F_3.*ta_3); %x1..
dx(:,27) =  y3dot; %y1.
dx(:,28) =  (1/m_ag)*(u_y3 -F_3.*tb_3); %y1..
dx(:,29) =  z3dot; %z1.
dx(:,30) =  (1/m_ag)*(u_z3 -F_3 -m_ag*g); %z1..

dx(:,31) =  xc3dot;%xcom.
dx(:,32) =  (1/m_pl)*Fxc;%xcom..
dx(:,33) =  yc3dot;%ycom.
dx(:,34) = (1/m_pl)*Fyc;%ycom..
dx(:,35) = zc3dot;%zcom.
dx(:,36) = (1/m_pl)*Fzc-g;%zcom..
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
x2 = x(:,13);
y2 = x(:,15); 
z2 = x(:,17);
x3 = x(:,25);
y3 = x(:,27); 
z3 = x(:,29);

xc1 = x(:,7);
yc1 = x(:,9);
zc1 = x(:,11);
xc2 = x(:,19);
yc2 = x(:,21);
zc2 = x(:,23);
xc3 = x(:,31);
yc3 = x(:,33);
zc3 = x(:,35);

sep1=sqrt((xc1-x1).^2+(yc1-y1).^2+(zc1-z1).^2);
sep2=sqrt((xc2-x2).^2+(yc2-y2).^2+(zc2-z2).^2);
sep3=sqrt((xc3-x3).^2+(yc3-y3).^2+(zc3-z3).^2);
c1 = sqrt((xc1-xc2).^2+(yc1-yc2).^2+(zc1-zc2).^2);
c2 = sqrt((xc3-xc2).^2+(yc3-yc2).^2+(zc3-zc2).^2);
c3 = sqrt((xc1-xc3).^2+(yc1-yc3).^2+(zc1-zc3).^2);
obstacle_height = 0;
obs_start = 3.1;
obs_end = 4.1;
o1 = zc1 - obstacle_height*(tanh(xc1-obs_start)-tanh(xc1-obs_end));
o2 = zc2 - obstacle_height*(tanh(xc2-obs_start)-tanh(xc2-obs_end));
o3 = zc3 - obstacle_height*(tanh(xc3-obs_start)-tanh(xc3-obs_end));

c=[abs(x1-x2), abs(x2-x3), abs(x1-x3), c1+c2-c3, sep1-1.2, sep2-1.2, sep3-1.2,o1,o2,o3];
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