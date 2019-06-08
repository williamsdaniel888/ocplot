function dx = MAGPlant(x,u,p,t,vdat)
% MAGPlant - Plant file for Adigator
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

Fxc = (F_1*ta_1+F_2*ta_2+F_3*ta_3);
Fyc = (F_1*tb_1+F_2*tb_2+F_3*tb_3);%
Fzc = (F_1+F_2+F_3); %

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