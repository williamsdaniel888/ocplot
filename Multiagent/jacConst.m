
function [df,dg,db]=jacConst(f,g,X,U,P,t,b,x0,xf,u0,uf,p,tf,t0,data)

%  jacConst - Return the gradient of the plant model, path constraints and  boundary constraints 
%
% Syntax: [df,dg,db]=jacConst(f,g,X,U,P,t,b,x0,xf,u0,uf,p,tf,t0,data)
%
% Inputs:
%    see directCollocation.m
%    Notice that the i-th state and input xi and ui, evaluated at the time instants t=[t0,...tk,...tf], 
%    are column  vectors taken as X(:,i) and U(:,i)
%
% Outputs:
%     df - Gradient of plant model with respect to  t, p, x, u
%     dg - Gradient of the path constraints with respect to t, p, x, u
%     db - Gradient of the boundary constraints with respect to  tf,p,x0,u0,uf,xf 
%    
%     Gradient df: 
%     df.flag - Set  df.flag=1 if the analytic form for df is supplied otherwise set df.flag=0; 
%     df.dx - Gradient of f(x,u,p,t)  wrt. x
%     df.du - Gradient of f(x,u,p,t)  wrt. u
%     df.dt - Gradient of f(x,u,p,t)  wrt. t
%     df.dp - Gradient of f(x,u,p,t)  wrt. p
%     If df.flag=1, the gradients have to be set considering the following rules:
%       1) If tf is a variable of the problem it is necessary to
%         specify  the derivative of f with respect to time t otherwise it possible to set df.dt=[].
%       2) If p=[p1, p2, ....,pn] is a variable of the problem it is necessary to
%         specify  the derivatives of f with respect to parameters pi otherwise it possible to set df.dp=[].  
%       3) df.dx and df.du have to be specified 
%       4) df.dx, df.du,  df.dp and df.dt are cell arrays and  must be vectorized.
%          For instance df.dx{i} contains the derivative  of f(x,u,p,t) with respect to the i-th
%          state variable xi. df.dx{i} is a matrix with n column where n is the dimension of the system.        
%          Each row of df.dx{i} stores the derivative of f(x,u,p,t)  (composed by  f1, f2, ....fn) 
%          evaluated at a time instant. The derivative of fj with respect to the i-th state variable, 
%          evaluated along all the horizon, corresponds to the i-th column of df.dx{i}(:,j);
%          
%     Gradient dg: 
%     dg.flag - Set  dg.flag=1 if the analytic form for dg is supplied otherwise set dg.flag=0; 
%     dg.dx - Gradient of g(x,u,p,t)  wrt. x
%     dg.du - Gradient of g(x,u,p,t)  wrt. u
%     dg.dt - Gradient of g(x,u,p,t)  wrt. t
%     dg.dp - Gradient of g(x,u,p,t)  wrt. p
%     If dg.flag=1, the gradients have to be set following the same rules explained for df.
%     Here the cell arrays  df.dx{i} is a matrix with ng column where ng is the number of path 
%     constraints. 
%
%     Gradient db;
%     db.flag - Set  db.flag=1 if the analytic form for db is supplied otherwise set db.flag=0; 
%     db.dtf  - Gradient of b(x0,xf,u0,uf,p,tf)  wrt. tf
%     db.dp   - Gradient of b(x0,xf,u0,uf,p,tf)  wrt. p
%     db.dx0  - Gradient of b(x0,xf,u0,uf,p,tf)  wrt. x0
%     db.du0  - Gradient of b(x0,xf,u0,uf,p,tf)  wrt. u0
%     db.dxf  - Gradient of b(x0,xf,u0,uf,p,tf)  wrt. xf
%     db.duf  - Gradient of b(x0,xf,u0,uf,p,tf)  wrt. uf
%     If the derivative of b does not depend on some variable it is possible to set
%     the respective variable as the empty matrix. For instance if it does not depend on tf set
%     db.dtf=[]; The gradient of b with respect to a vector variable  has to be specified in a matrix
%     with dimension nb x s where s is the size of the vector variable. For instance db.dxf has to be a matrix 
%     with dimension nb x n and the entry (i,j) correspond to the derivative of th i-th constraints 
%     with respect of the j-th state variable in xf.
%     
% 
%
% Other m-files required: none
%         Subfunctions: none
% MAT-files required: none
%
% Copyright (C) 2010 Paola Falugi, Eric Kerrigan and Eugene van Wyk. All Rights Reserved.
% This code is published under the BSD License.
% Department of Electrical and Electronic Engineering,
% Imperial College London London  England, UK 
% ICLOCS (Imperial College London Optimal Control) 5 May 2010
% iclocs@imperial.ac.uk

%------------- BEGIN CODE --------------



%[nt,np,n,m]=deal(data.sizes{1:4});   % extract variable dimension
% if nt is 1 the vector df.dt can not be empty


Lt=ones(size(t));
df.flag=0;               % df.flag=1 if the analytic is supplied otherwise set
                         % df.flag=0;              
% df.dp=[];             % Derivatives of f(x,u,p,t) wrt. p
% df.dt=[];             % Derivatives of f(x,u,p,t) wrt. t
% df.dx{1}=[0.*Lt,0.*Lt,0.*Lt,0.*Lt,0.*Lt,0.*Lt]; % Derivatives of f(x,u,p,t) wrt. x1
% df.dx{2}=[1.*Lt,0.*Lt,0.*Lt,0.*Lt,0.*Lt,0.*Lt]; % Derivative of f(x,u,p,t) wrt. x2
% df.dx{3}=[0.*Lt,0.*Lt,0.*Lt,0.*Lt,0.*Lt,0.*Lt]; % Derivative of f(x,u,p,t) wrt. x3
% df.dx{4}=[0.*Lt,0.*Lt,1.*Lt,0.*Lt,0.*Lt,0.*Lt]; % Derivative of f(x,u,p,t) wrt. x4
% df.dx{5}=[0.*Lt,0.*Lt,0.*Lt,0.*Lt,0.*Lt,0.*Lt]; % Derivative of f(x,u,p,t) wrt. x5
% df.dx{6}=[0.*Lt,0.*Lt,0.*Lt,0.*Lt,1.*Lt,0.*Lt]; % Derivative of f(x,u,p,t) wrt. x6
% df.du{1}=[0.*Lt,1.*Lt,0.*Lt,0.*Lt,0.*Lt,0.*Lt];  % Derivative of f(x,u,p,t) wrt. u1
% df.du{2}=[0.*Lt,0.*Lt,0.*Lt,1.*Lt,0.*Lt,0.*Lt];  % Derivative of f(x,u,p,t) wrt. u2
% df.du{3}=[0*Lt,0*Lt,0*Lt,0*Lt,0*Lt,1*Lt];  % Derivative of f(x,u,p,t) wrt. u3

dg.flag=0;
% k = 1;
% h = 7;
% wos = 9;
% woe = 12;
% dg.flag=1;
% dg.dp=[];             % Derivatives of g(x,u,p,t) wrt. p
% dg.dt=[];             % Derivatives of g(x,u,p,t) wrt. t
% % dg.dx{1}=(2/45)*X(:,1)-2/3;
% % dg.dx{1}=-2.*X(:,1)+30; % Derivatives of g(x,u,p,t) wrt. x1
% % dg.dx{2}=0.*Lt; % Derivative of g(x,u,p,t) wrt. x2
% % dg.dx{3}=0.*Lt; % Derivative of g(x,u,p,t) wrt. x3
% % dg.dx{4}=0.*Lt; % Derivative of g(x,u,p,t) wrt. x4
% % dg.dx{5}=1.*Lt; % Derivative of g(x,u,p,t) wrt. x5
% % dg.dx{6}=0.*Lt; % Derivative of g(x,u,p,t) wrt. x6
% % dg.du{1}=0.*Lt;  % Derivative of g(x,u,p,t) wrt. u1
% % dg.du{2}=0.*Lt;  % Derivative of g(x,u,p,t) wrt. u2
% % dg.du{3}=0.*Lt;  % Derivative of g(x,u,p,t) wrt. u3
% % dg.flag=1;
% % dg.dp=[];             % Derivatives of g(x,u,p,t) wrt. p
% % dg.dt=[];             % Derivatives of g(x,u,p,t) wrt. t
% dg.dx{1}= -k*h*0.5*((sech(k*(X(:,1)-wos))).^2 - (sech(k*(X(:,1)-woe))).^2); % Derivatives of g(x,u,p,t) wrt. x1
% dg.dx{2}=0.*Lt; % Derivative of g(x,u,p,t) wrt. x2
% dg.dx{3}=0.*Lt; % Derivative of g(x,u,p,t) wrt. x3
% dg.dx{4}=0.*Lt; % Derivative of g(x,u,p,t) wrt. x4
% dg.dx{5}= 1*Lt; % Derivative of g(x,u,p,t) wrt. x5
% dg.dx{6}=0.*Lt; % Derivative of g(x,u,p,t) wrt. x6
% dg.du{1}=0.*Lt;  % Derivative of g(x,u,p,t) wrt. u1
% dg.du{2}=0.*Lt;  % Derivative of g(x,u,p,t) wrt. u2
% dg.du{3}=0.*Lt;  % Derivative of g(x,u,p,t) wrt. u3

db.flag=0;
%------------- END CODE --------------