function dx = MAXPlant(x,u,p,t,vdat)
% MAXPlant - Plant model for Adigator
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

m_ag = 0.81;
m_pl = 2;
g = 9.81;

u_x1=u(:,1);
u_y1=u(:,2);
u_z1=u(:,3);
F_1 = u(:,4);

x1dot = x(:,2);
y1dot = x(:,4);
z1dot = x(:,6);
xcomdot = x(:,8);
ycomdot = x(:,10);
zcomdot = x(:,12);
ta_1 = x(:,13);
tb_1 = x(:,14);

dx(:,1) =  x1dot; %x1.
dx(:,2) =  (1/m_ag)*(u_x1 -F_1.*ta_1); %x1..
dx(:,3) =  y1dot; %x(:,3).
dx(:,4) =  (1/m_ag)*(u_y1 -F_1.*tb_1); %y1..
dx(:,5) =  z1dot; %z1.
dx(:,6) =  (1/m_ag)*(u_z1 -F_1 -m_ag*g); %z1..
dx(:,7) =  xcomdot;%xcom.
dx(:,8) =  (1/m_pl)*(F_1.*ta_1);%xcom..
dx(:,9) =  ycomdot;%ycom.
dx(:,10) = (1/m_pl)*(F_1.*tb_1);%ycom..
dx(:,11) = zcomdot;%zcom.
dx(:,12) = (1/m_pl)*(F_1 - m_pl.*g);%zcom..
dx(:,13) = ((x(:,5)-x(:,11)).*(x(:,2)-x(:,8))-(x(:,1)-x(:,7)).*(x(:,6)-x(:,12)))./((x(:,5)-x(:,11)).^2);%ta1.
dx(:,14) = ((x(:,5)-x(:,11)).*(x(:,4)-x(:,10))-(x(:,3)-x(:,9)).*(x(:,6)-x(:,12)))./((x(:,5)-x(:,11)).^2);%tb1.

%------------- END OF CODE --------------