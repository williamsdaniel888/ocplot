function [problem,guess] = MultiAgentQUAVProblemNewtRestricted
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
problem.data.plantmodel = 'MultiAgentQUAVPlantNewtRestricted';

 
% Initial time. t0<tf
problem.time.t0=0;

% Final time. Let tf_min=tf_max if tf is fixed.
problem.time.tf_min=20;     
problem.time.tf_max=20; 
guess.tf=20;

% Plant parameters
m_ag = 0.81;
m_pl = 20;
gc = 9.81;
% Parameters bounds. pl=< p <=pu
problem.parameters.pl=[];
problem.parameters.pu=[];
guess.parameters=[];

% Initial conditions for system
problem.states.x0=[2.1 0 0 0 2.2 0 3 0 0 0 1 0, 3 0 0 0 2.2 0 3.9 0 0 0 2.2 0];

% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
problem.states.x0l=[2.1 0 0 0 2.2 0 3 0 0 0 1 0, 3 0 0 0 2.2 0 3.9 0 0 0 2.2 0];
problem.states.x0u=[2.1 0 0 0 2.2 0 3 0 0 0 1 0, 3 0 0 0 2.2 0 3.9 0 0 0 2.2 0];

% State bounds. xl=< x <=xu
problem.states.xl=[0 -20 0 0 2.2 -20 0 0 0 0 0.5 -20, 0 -20 0 0 2.2 -20 0 -20 0 0 2.2 -20];
problem.states.xu=[35 20 0 0 2.2 20 35 20 0 0 1.5 20, 35 20 0 0 2.2 20 35 20 0 0 2.2 20];

% State error bounds
problem.states.xErrorTol=(1e-1)*ones(1,24);

% State constraint error bounds
problem.states.xConstraintTol=(1e-1)*ones(1,24);

% Terminal state bounds. xfl=< xf <=xfu
problem.states.xfl=[5.1 -20 0 0 2.2 -20 0 -20 0 0 1 -20,6 -20 0 0 2.2 -20 6.9 -20 0 0 2.2 -20 ];
problem.states.xfu=[5.1 20 0 0 2.2 20 12 20 0 0 1 20,   6 20 0 0 2.2 20 6.9 0 20 0 2.2 20 ];

% Guess the state trajectories with [x0 xf]
guess.states(:,1)=[2.1 5.1];
guess.states(:,2)=[0 0];
guess.states(:,3)=[0 0];
guess.states(:,4)=[0 0];
guess.states(:,5)=[2.2 2.2];
guess.states(:,6)=[0 0];
guess.states(:,7)=[3 6];
guess.states(:,8)=[0 0];
guess.states(:,9)=[0 0];
guess.states(:,10)=[0 0];
guess.states(:,11)=[1 1];
guess.states(:,12)=[0 0];
guess.states(:,13)=[3 6];
guess.states(:,14)=[0 0];
guess.states(:,15)=[0 0];
guess.states(:,16)=[0 0];
guess.states(:,17)=[2.2 2.2];
guess.states(:,18)=[0 0];
guess.states(:,19)=[3.9 6.9];
guess.states(:,20)=[0 0];
guess.states(:,21)=[0 0];
guess.states(:,22)=[0 0];
guess.states(:,23)=[2.2 2.2];
guess.states(:,24)=[0 0];

% Number of control actions N 
% Set problem.inputs.N=0 if N is equal to the number of integration steps.  
% Note that the number of integration steps defined in settings.m has to be divisible 
% by the  number of control actions N whenever it is not zero.

problem.inputs.N=0;
% Bounds on first control action
cc = (m_ag+m_pl/3)*gc;
problem.inputs.u0l=[-10 -10 0,  m_pl*gc/3 m_pl*gc/3  m_pl*gc/3   0 0 0 0 0 0, -10 -10 0 -10 -10 0];%[-10 -10 0 m_pl*gc/3 m_pl*gc/3 m_pl*gc/3 0 0 0 0 0 0, -10 -10 0 -10 -10 0];
problem.inputs.u0u=[ 10 10 cc,  m_pl*gc/3 m_pl*gc/3  m_pl*gc/3   0 0 0 0 0 0, 10 10 cc 10 10 cc];

% Input bounds
problem.inputs.ul=[-10 -10 0,m_pl*gc/3         m_pl*gc/3   m_pl*gc/3    -0.75 -0.75 -0.75 -0.75 -0.75 -0.75, -10 -10 0 -10 -10 0];
problem.inputs.uu=[10 10 cc, m_pl*gc/3         m_pl*gc/3   m_pl*gc/3    0.75  0.75  0.75  0.75  0.75  0.75, 10 10 cc 10 10 cc];%[10 10 cc, m_pl*gc   m_pl*gc   m_pl*gc   0.5  0.5  0.5  0.5  0.5  0.5, 10 10 cc 10 10 cc];


% Input constraint error bounds
problem.inputs.uConstraintTol= 0.1*ones(1,18);

% Guess the input sequences with [u0 uf]
guess.inputs(:,1)=[1 -1]; %u_x1
guess.inputs(:,2)=[0 0];
guess.inputs(:,3)=[cc cc];
guess.inputs(:,4)=[m_pl*gc/3 m_pl*gc/3]; %F_1
guess.inputs(:,5)=[m_pl*gc/3 m_pl*gc/3]; %F_2
guess.inputs(:,6)=[m_pl*gc/3 m_pl*gc/3]; %F_3
guess.inputs(:,7)=[-0.75 0.75]; %t_a1
guess.inputs(:,8)=[-0.75 0.75];
guess.inputs(:,9)=[-0.75 0.75]; %t_a2
guess.inputs(:,10)=[-0.75 0.75];
guess.inputs(:,11)=[-0.75 0.75]; %t_a3
guess.inputs(:,12)=[-0.75 0.75];
guess.inputs(:,13)=[1 -1]; %u_x2
guess.inputs(:,14)=[0 0];
guess.inputs(:,15)=[cc cc];
guess.inputs(:,16)=[1 -1]; %u_x3
guess.inputs(:,17)=[0 0];
guess.inputs(:,18)=[cc cc];

% Choose the set-points if required
problem.setpoints.states=[];
problem.setpoints.inputs=[];

% Bounds for path constraint function gl =< g(x,u,p,t) =< gu
problem.constraints.gl=[0,0,0,0,0,0,0];
problem.constraints.gu=[0,0,0,0,0,0,0];
problem.constraints.gTol=(1e-2)*ones(1,7);

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
% u1 = u(:,1);
% u2 = u(:,2);
% u3 = u(:,3);
% stageCost = u1.*u1+u2.*u2+u3.*u3;

u1 = u(:,1);
u2 = u(:,2);
u3 = u(:,3);
u4 = u(:,13);
u5 = u(:,14);
u6 = u(:,15);
u7 = u(:,16);
u8 = u(:,17);
u9 = u(:,18);

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
m_pl = 20;
g = 9.81;

u_x1=u(:,1);
u_y1=u(:,2);
u_z1=u(:,3);
F_1 = u(:,4);
F_2 = u(:,5);
F_3 = u(:,6);
ta_1 = u(:,7);
tb_1 = u(:,8);
ta_2 = u(:,9);
tb_2 = u(:,10);
ta_3 = u(:,11);
tb_3 = u(:,12);
u_x2=u(:,13);
u_y2=u(:,14);
u_z2=u(:,15);
u_x3=u(:,16);
u_y3=u(:,17);
u_z3=u(:,18);

x1dot = x(:,2);
y1dot = x(:,4);
z1dot = x(:,6);
xcomdot = x(:,8);
ycomdot = x(:,10);
zcomdot = x(:,12);
x2dot = x(:,14);
y2dot = x(:,16);
z2dot = x(:,18);
x3dot = x(:,20);
y3dot = x(:,22);
z3dot = x(:,24);

dx(:,1) =  x1dot; %x1.
dx(:,2) =  (1/m_ag)*(u_x1 -F_1.*ta_1); %x1..
dx(:,3) =  y1dot; %y1.
dx(:,4) =  (1/m_ag)*(u_y1 -F_1.*tb_1); %y1..
dx(:,5) =  z1dot; %z1.
dx(:,6) =  (1/m_ag)*(u_z1 -F_1 -m_ag*g); %z1..
dx(:,7) =  xcomdot;%xcom.
dx(:,8) =  (1/m_pl)*(F_1.*ta_1+F_2.*ta_2+F_3.*ta_3);%xcom..
dx(:,9) =  ycomdot;%ycom.
dx(:,10) = (1/m_pl)*(F_1.*tb_1+F_2.*tb_2+F_3.*tb_3);%ycom..
dx(:,11) = zcomdot;%zcom.
dx(:,12) = (1/m_pl)*(F_1+F_2+F_3 - m_pl.*g);%zcom..

dx(:,13) =  x2dot; %x2.
dx(:,14) =  (1/m_ag)*(u_x2 -F_2.*ta_2); %x2..
dx(:,15) =  y2dot; %y2.
dx(:,16) =  (1/m_ag)*(u_y2 -F_2.*tb_2) ;%y2..
dx(:,17) =  z2dot; %z2.
dx(:,18) =  (1/m_ag)*(u_z2 - F_2 - m_ag*g);%z2..

dx(:,19) =  x3dot; %x3.
dx(:,20) =  (1/m_ag)*(u_x3 -F_3.*ta_3) ;%x3..
dx(:,21) =  y3dot; %y3.
dx(:,22) =  (1/m_ag)*(u_y3 -F_3.*tb_3) ;%y3..
dx(:,23) =  z3dot; %z3.
dx(:,24) =  (1/m_ag)*(u_z3  -F_3 -m_ag*g);%z3..


% m_ag = 0.81;
% g = 9.81;
% 
% u_x1=u(:,1);
% u_y1=u(:,2);
% u_z1=u(:,3);
% 
% x1dot = x(:,2);
% y1dot = x(:,4);
% z1dot = x(:,6);
% 
% dx(:,1) = x1dot; %x1.
% dx(:,2) = u_x1 ; %x1..
% dx(:,3) = y1dot; %y1.
% dx(:,4) = u_y1 ; %y1..
% dx(:,5) = z1dot; %z1.
% dx(:,6) = u_z1 - m_ag.*g;%z1..

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
m_pl = 20;
g = 9.81;

F_1 = u(:,4);
F_2 = u(:,5);
F_3 = u(:,6);
ta_1 = u(:,7);
tb_1 = u(:,8);
ta_2 = u(:,9);
tb_2 = u(:,10);
ta_3 = u(:,11);
tb_3 = u(:,12);

x1 = x(:,1);
y1 = x(:,3);
z1 = x(:,5);
xcom = x(:,7);
ycom = x(:,9);
zcom = x(:,11);
x2 = x(:,13);
y2 = x(:,15);
z2 = x(:,17);
x3 = x(:,19);
y3 = x(:,21);
z3 = x(:,23);

ac_1 = x1 - (xcom-0.9);
ac_2 = x2 - xcom;
ac_3 = x3 - (xcom+0.9);
bc_1 = y1 - ycom;
bc_2 = y2 - ycom;
bc_3 = y3 - ycom;
cc_1 = z1 - (zcom+0.05);
cc_2 = z2 - (zcom+0.05);
cc_3 = z3 - (zcom+0.05);

c=[ta_1-ac_1./cc_1, tb_1-bc_1./cc_1, ta_2-ac_2./cc_2, tb_2-bc_2./cc_2, ta_3-ac_3./cc_3, tb_3-bc_3./cc_3,   F_1+F_2+F_3-m_pl*g];

% m_pl = 20;
% g = 9.81;
% 
% F_1 = u(:,10);
% F_2 = u(:,11);
% F_3 = u(:,12);
% ta_1 = u(:,13);
% tb_1 = u(:,14);
% ta_2 = u(:,15);
% tb_2 = u(:,16);
% ta_3 = u(:,17);
% tb_3 = u(:,18);
% 
% x1 = x(:,1);
% y1 = x(:,3);
% z1 = x(:,5);
% % x2 = x(:,7);
% % y2 = x(:,9);
% % z2 = x(:,11);
% % x3 = x(:,13);
% % y3 = x(:,15);
% % z3 = x(:,17);
% xcom = x(:,19);
% ycom = x(:,21);
% zcom = x(:,23);
% 

% 
% c=[F_1+F_2+F_3-m_pl*g, ta_1-ac_1./cc_1, tb_1-bc_1./cc_1, ta_2, tb_2, ta_3, tb_3];

%problematic
%RETHINK NEED TO PERFORM ALL OF THESE CALCULATIONS - ISSUE WITH RESHAPE
% N = size(phi,1);

% Rphi = kron(eye(N),[1,0,0;0,0,0;0,0,0]) + kron(eye(N),[0,0,0;0,1,0;0,0,1])*kron(diag(cos(phi)),eye(3))+kron(eye(N),[0,0,0;0,0,-1;0,1,0])*kron(diag(sin(phi)),eye(3));
% % Rphi = [1,0,0;0,cos(phi),-sin(phi);0,sin(phi),cos(phi)];
% Rtheta = kron(eye(N),[0,0,0;0,1,0;0,0,0]) + kron(eye(N),[1,0,0;0,0,0;0,0,1])*kron(diag(cos(theta)),eye(3))+kron(eye(N),[0,0,1;0,0,0;-1,0,0])*kron(diag(sin(theta)),eye(3));
% % Rtheta = [cos(theta),0,sin(theta);0,1,0;-sin(theta),0,cos(theta)];
% Rpsi = kron(eye(N),[0,0,0;0,0,0;0,0,1]) + kron(eye(N),[1,0,0;0,1,0;0,0,0])*kron(diag(cos(psi)),eye(3)) + kron(eye(N),[0,-1,0;1,0,0;0,0,0])*kron(diag(sin(psi)),eye(3));
% % Rpsi = [cos(psi),-sin(psi),0;sin(psi),cos(psi),0;0,0,1];
% R = kron(ones(1,N),eye(3))*Rphi*Rtheta*Rpsi; %Rphi*Rtheta*Rpsi;

% x_d1 = -0.9;
% y_d1 = 0.4;
% z_d1 = 0.05;
% x_d2 = 0.2;
% y_d2 =-0.3;
% z_d2 = 0.05;
% x_d3 = 0.8;
% y_d3 = 0.2;
% z_d3 = 0.05;
% pd1 = [x_d1,y_d1,z_d1];
% pd2 = [x_d2,y_d2,z_d2];
% pd3 = [x_d3,y_d3,z_d3];
% pcom = [xcom';ycom';zcom']';%kron(xcom,[1;0;0])+kron(ycom,[0;1;0])+kron(zcom,[0;0;1]);%[xcom;ycom;zcom];
% p1 = pcom + kron(ones(N,1),pd1); %(R*kron(ones(N,N),pd1))';%R*pd1;%
% p2 = pcom + kron(ones(N,1),pd2); %(R*kron(ones(N,N),pd2))';%R*pd2;%
% p3 = pcom + kron(ones(N,1),pd3); %(R*kron(ones(N,N),pd3))';%R*pd3;%

% ac_1 = x1 - xcom;%p1*[1; 0; 0];%dot([1 0 0],p1);%[1 0 0]*p1;%
% ac_2 = x2 - xcom;%p2*[1; 0; 0];%dot([1 0 0],p2);%[1 0 0]*p2;%
% ac_3 = x3 - xcom;%p3*[1; 0; 0];%dot([1 0 0],p3);%[1 0 0]*p3;%
% bc_1 = y1 - ycom;%p1*[0; 1; 0];%dot([0 1 0],p1);%[0 1 0]*p1;%
% bc_2 = y2 - ycom;%p2*[0; 1; 0];%dot([0 1 0],p2);%[0 1 0]*p2;%
% bc_3 = y3 - ycom;%p3*[0; 1; 0];%dot([0 1 0],p3);%[0 1 0]*p3;%
% cc_1 = z1 - zcom;%p1*[0; 0; 1];%;dot([0 0 1],p1);%[0 0 1]*p1;%
% cc_2 = z2 - zcom;%p2*[0; 0; 1];%dot([0 0 1],p2);%[0 0 1]*p2;%
% cc_3 = z3 - zcom;%p3*[0; 0; 1];%dot([0 0 1],p3);%[0 0 1]*p3;%

% u = p1-p2;
% v = p1-p3;
% Nr = cross(u,v)';
% Nx = Nr(1,:);
% Ny = Nr(2,:);
% Nz = Nr(3,:);

% c=[F_1+F_2+F_3-m_pl*g, phi-atan(Ny./Nx), theta-atan(Nz./Nx), psi-atan(Nz./Ny), a_1-atan(ac_1./cc_1), b_1-atan(bc_1./cc_1), a_2-atan(ac_2./cc_2), b_2-atan(bc_2./cc_2), a_3-atan(ac_3./cc_3), b_3-atan(bc_3./cc_3)];
% c=[F_1+F_2+F_3-m_pl*g, ta_1-ac_1./cc_1, tb_1-bc_1./cc_1, ta_2-ac_2./cc_2, tb_2-bc_2./cc_2, ta_3-ac_3./cc_3, tb_3-bc_3./cc_3];
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