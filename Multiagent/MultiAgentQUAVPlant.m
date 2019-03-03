
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