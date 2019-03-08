function dx = MultiAgentQUAVPlantNewtRestricted(x,u,p,t,vdat)
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
m_pl = 20;
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
F_1 = u(:,10);
F_2 = u(:,11);
F_3 = u(:,12);
ta_1 = u(:,13);
tb_1 = u(:,14);
ta_2 = u(:,15);
tb_2 = u(:,16);
ta_3 = u(:,17);
tb_3 = u(:,18);

x1dot = x(:,2);
y1dot = x(:,4);
z1dot = x(:,6);
x2dot = x(:,8);
y2dot = x(:,10);
z2dot = x(:,12);
x3dot = x(:,14);
y3dot = x(:,16);
z3dot = x(:,18);
xcomdot = x(:,20);
ycomdot = x(:,22);
zcomdot = x(:,24);

dx(:,1) = x1dot; %x1.
dx(:,2) = (u_x1 - F_1.*ta_1); %x1..
dx(:,3) = y1dot; %y1.
dx(:,4) = (u_y1 - F_1.*tb_1); %y1..
dx(:,5) = z1dot; %z1.
dx(:,6) = (u_z1 - F_1 - m_ag.*g); %z1..
dx(:,7) = x2dot; %x2.
dx(:,8) = (u_x2 - F_2.*ta_2); %x2..
dx(:,9) = y2dot; %y2.
dx(:,10) = (u_y2 - F_2.*tb_2); %y2..
dx(:,11) = z2dot; %z2.
dx(:,12) = (u_z2 - F_2 - m_ag.*g); %z2..
dx(:,13) = x3dot; %x3.
dx(:,14) = (u_x3 - F_3.*ta_3); %x3..
dx(:,15) = y3dot; %y3.
dx(:,16) = (u_y3 - F_3.*tb_3); %y3..
dx(:,17) = z3dot; %z3.
dx(:,18) = (u_z3 - F_3 - m_ag.*g); %z3..
dx(:,19) =  xcomdot;%xcom.
dx(:,20) =  (F_1.*ta_1 + F_2.*ta_2 + F_3.*ta_3);%xcom..
dx(:,21) =  ycomdot;%ycom.
dx(:,22) =  (F_1.*tb_1 + F_2.*tb_2 + F_3.*tb_3);%ycom..
dx(:,23) =  zcomdot;%zcom.
dx(:,24) =  (F_1 + F_2 + F_3 - m_pl.*g);%zcom..

% m_ag = 0.81;
% m_pl = 20;
% g = 9.81;
% 
% u_x1=u(:,1);
% u_y1=u(:,2);
% u_z1=u(:,3);
% F_1 = u(:,4);
% ta_1 = u(:,5);
% tb_1 = u(:,6);
% 
% x1dot = x(:,2);
% y1dot = x(:,4);
% z1dot = x(:,6);
% xcomdot = x(:,8);
% ycomdot = x(:,10);
% zcomdot = x(:,12);
% 
% dx(:,1) =  x1dot; %x1.
% dx(:,2) =  u_x1 ;%- F_1.*ta_1); %x1..
% dx(:,3) =  y1dot; %y1.
% dx(:,4) =  u_y1 ;%- F_1.*tb_1); %y1..
% dx(:,5) =  z1dot; %z1.
% dx(:,6) =  u_z1 -m_ag.*g;%- F_1 - m_ag.*g); %z1..
% dx(:,7) =  xcomdot;%xcom.
% dx(:,8) =  F_1.*ta_1;%xcom..
% dx(:,9) =  ycomdot;%ycom.
% dx(:,10) = F_1.*tb_1;%ycom..
% dx(:,11) = zcomdot;%zcom.
% dx(:,12) = F_1 - m_pl.*g;%zcom..

% m_ag = 0.81;%p(1,1);
% g = 20;%p(1,3);
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