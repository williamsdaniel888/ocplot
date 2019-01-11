
function dx = SingleAgentQUAVPlant(x,u,p,t,vdat)
%Double Integrator Dynamics
%
% Copyright (C) 2018 Yuanbo Nie, Omar Faqir, and Eric Kerrigan. All Rights Reserved.
% The contribution of Paola Falugi, Eric Kerrigan and Eugene van Wyk for the work on ICLOCS Version 1 (2010) is kindly acknowledged.
% This code is published under the BSD License.
% Department of Aeronautics and Department of Electrical and Electronic Engineering,
% Imperial College London London  England, UK 
% ICLOCS (Imperial College London Optimal Control) Version 2.0 
% 1 May 2018
% iclocs@imperial.ac.uk
%
%------------- BEGIN CODE --------------

x2 = x(:,2);
x4 = x(:,4);
x6 = x(:,6);
u1 = u(:,1);
u2 = u(:,2);
u3 = u(:,3);

dx(:,1) = x2;
dx(:,2) = u1;
dx(:,3) = x4;
dx(:,4) = u2;
dx(:,5) = x6;
dx(:,6) = u3-9.81;

%------------- END OF CODE --------------