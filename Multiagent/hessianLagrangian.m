
function [HL,HE,Hf,Hg,Hb]=hessianLagrangian(X,U,P,t,E,x0,xf,u0,uf,p,tf,data)

%  hessianLagrangian - Return the Hessian of the Lagrangian
%
% Syntax:  [HL,HE,Hf,Hg,Hb]=hessianLagrangian(X,U,P,t,E,x0,xf,u0,uf,p,tf,data)
%
% Inputs:
%    see directCollocation.m
%
% Outputs:
%    HL -  hessian of L wrt wz or a subset of its variables
%    HE -  hessian of E wrt [tf p x0 u0 uf xf] or a subset of its variables
%    Hf -  hessian of f wrt  wz or a subset of its variables
%    Hg -  hessian of g wrt wz or a subset of its variables
%    Hb -  hessian of b wrt [tf p x0 u0 uf xf]  or a subset of its variables
%
%   If the hessian of some component of the Lagrangian is not available set
%   the corresponding output term equal to the empty matrix. 
%   For instance if the hessian of the dynamical system is not available set Hf=[]; 
%   If  some variables do not contribute to the Hessian, the corresponding entries  can be set  to zero, otherwise the entries 
%   have to be vectors with the same length of t, containing the value of the Hessian at different  time instants.
%
% The vector wz is defined as wz=[t, p, x, u]. The variables
% have the following meaning:
%
%    t - denotes the time variable. The hessian with respect to the time t must be considered only if 
%        the final time tf is a decision variable of the optimal control problem 
%    p - parameters.  The hessian with respect to the  p has to be
%        considered only if the optimal control problem depends on some
%        constant parameter that is an optimization variable
%    x - denotes the state variable x=[x1, x2,   xn] where n is the
%        dimension of the state equation.
%    u - denotes the input variable u=[u1, u2,   um] where m is the number
%        of the control inputs
%   
%    Each hessian has to be specified in a cell array of dimension depending on the variables in wz 
%    or in [tf p x0 u0 uf xf] considering the following rules. 
%    The order to follow is  given by the order of the vector variables inside wz=[t p x u] or [tf p x0 u0 uf xf]. 
%    
%    Hessian wrt wz or a subset of its variables: 
%    If tf (or/and p) is not a decision variable the derivative with respect to time (or/and p) must 
%    not be considered i.e the Hessian must be computed wrt [p x u] or ([t x u] or [x u]) 
%    
%    Hessian wrt  [tf p x0 u0 uf xf] or a subset of its variables: 
%    The Hessian with respect to the variables tf, p, x0, u0, uf and xf has to be considered on the basis of the 
%    specified analytic gradient when its evaluation is enabled (flag=1).
%    For instance if db.flag=1 and db.dtf, db.dp,  db.du0 and db.duf are  empty matrices and  db.dx0 and db.dxf are specified
%    (see  the file  jacConst.m) the Hessian must be specified considering the vector of decision variables [x0 xf].
%    Moreover if tf (or/and p) is not a decision variable (nt=0 (or/and np=0)) the derivative with respect to  
%    the final time (or/and p) must not be considered 
%    If  flag=0 the Hessian with respect to the variables x0, u0, uf and xf  must be always specified  
%     
%  
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% Copyright (C) 2010 Paola Falugi, Eric Kerrigan and Eugene van Wyk. All Rights Reserved.
% This code is published under the BSD License.
% Department of Electrical and Electronic Engineering,
% Imperial College London London  England, UK 
% ICLOCS (Imperial College London Optimal Control) 5 May 2010
% iclocs@imperial.ac.uk

%------------- BEGIN CODE ---------------


% Get some useful variables

[nt,np,n,m,ng,nb]=deal(data.sizes{1:6});
nfz=n+m+nt+np;
% 
% 
% 
% % Write the hessian for the system equations in a cell array with dimension
% % nfz*nfz 
% % The order to follow is  given by the order of the vector variable  wz=[t p x u]
% % where tf, p, x, and u are decision variables. 
% % If tf (or/and p) is not a decision variable the derivative with respect to time must not be considered i.e the vector is defined as wz=[p x u]  (wz=[tf x u] or wz=[x u]) 
% %    
% %
% 
Hf=[];
% Hf=cell(nfz, nfz);
% Hf(:)={[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t]}; 
% 
% Hf{1,1}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];   % [d^2f1/dx1^2, d^2f2/dx1^2, d^2f3/dx1^2, d^2f4/dx1^2  d^2f5/dx1^2 d^2f6/dx1^2]  (nt=0)
% Hf{1,2}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{1,3}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{1,4}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{1,5}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{1,6}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{1,7}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{1,8}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{1,9}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{2,2}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{2,3}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{2,4}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{2,5}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{2,6}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{2,7}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{2,8}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{2,9}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{3,3}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{3,4}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{3,5}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{3,6}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{3,7}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{3,8}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{3,9}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{4,4}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{4,5}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{4,6}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{4,7}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{4,8}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{4,9}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{5,5}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{5,6}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{5,7}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{5,8}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{5,9}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{6,6}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{6,7}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{6,8}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{6,9}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{7,7}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{7,8}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{7,9}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{8,8}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{8,9}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
% Hf{9,9}=[0.*t, 0.*t, 0.*t, 0.*t, 0.*t, 0.*t];
   
% %%%%%%%%%%%%%
%figure('Name',sprintf('For HL, %d + %d = %d',n,m,nfz));
% Lt=ones(size(t));
HL=cell(nfz, nfz);
HL(:) = {0};
HL{7,7}=2;
HL{8,8}=2;
HL{9,9}=2;

% HL=[];

%%%%%%%%%%%%%%%%%%%%%%

% nE=n;
% Ez=zeros(nE,nE);
% HE=num2cell(Ez);
% HE{2,nE}=-1;
% figure('Name',sprintf('For HE, %d + %d = %d',n,m,nfz));

%if dE.flag=0 the Hessian of the boundary cost has to be specified in the
% following way
%  
% nE=nt+np+2*n+2*m;
% Ez=zeros(nE,nE);
% HE=num2cell(Ez);

HE=cell(nfz, nfz);
HE(:)={0};
% HE=[];
% HE{8,nE}=-1;

%%%%%%%%
Hg = [];
% k = 1;
% h = 7;
% wos = 9;
% woe = 12;
% 
% Hg=cell(nfz, nfz);
% Hg(:) = {0.*t};
% % Hg{1,1}= -2.*ones(size(t));    % [d^2g/dx_5^2] (nt=0)
% % Hg{1,1}=(2/45).*ones(size(t));    % [d^2g/dx_5^2] (nt=0)
% Hg{1,1}= (k^2)*h*((sech(k*(X(:,1)-wos))).^2.*tanh(k*(X(:,1)-wos))-(sech(k*(X(:,1)-woe))).^2.*tanh(k*(X(:,1)-woe)));    % [d^2g/dx_1^2] (nt=0)

%%%%%%%%
Hb=[];











