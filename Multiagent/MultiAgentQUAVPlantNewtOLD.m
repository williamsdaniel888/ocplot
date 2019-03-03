
function dx = MultiAgentQUAVPlantNewt(x,u,p,t,vdat)
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

x1 = x(:,1);
x1dot = x(:,2);
y1 = x(:,3);
y1dot = x(:,4);
z1 = x(:,5);
z1dot = x(:,6);
x2 = x(:,7);
x2dot = x(:,8);
y2 = x(:,9);
y2dot = x(:,10);
z2 = x(:,11);
z2dot = x(:,12);
x3 = x(:,13);
x3dot = x(:,14);
y3 = x(:,15);
y3dot = x(:,16);
z3 = x(:,17);
z3dot = x(:,18);
xcom = x(:,19);
xcomdot = x(:,20);
ycom = x(:,21);
ycomdot = x(:,22);
zcom = x(:,23);
zcomdot = x(:,24);
F_1 = x(:,25);
F_2 = x(:,26);
F_3 = x(:,27);

phi = x(:,28);
phidot = x(:,29);
theta = x(:,30);
thetadot = x(:,31);
psi = x(:,32);
psidot = x(:,33);

% Problematic
a_1 = x(:,35);
b_1 = x(:,37);
a_2 = x(:,39);
b_2 = x(:,41);
a_3 = x(:,43);
b_3 = x(:,44);
% End Problematic

Rphi = [1,0,0;0,cos(phi),-sin(phi);0,sin(phi),cos(phi)];
Rphidot = [0,0,0;0,-sin(phi)*phidot,-cos(phi)*phidot;0,cos(phi)*phidot,-sin(phi)*phidot];
Rtheta = [cos(theta),0,sin(theta);0,1,0;-sin(theta),0,cos(theta)];
Rthetadot = [-sin(theta)*thetadot,0,cos(theta)*thetadot;0,1,0;-cos(theta)*thetadot,0,-sin(theta)*thetadot];
Rpsi = [cos(psi),-sin(psi),0;sin(psi),cos(psi),0;0,0,1];
Rpsidot = [-sin(psi)*psidot,-cos(psi)*psidot,0;cos(psi)*psidot,-sin(psi)*psidot,0;0,0,0];
R = Rphi*Rtheta*Rpsi;
Rdot = Rphidot*Rtheta*Rpsi + Rphi*Rthetadot*Rpsi + Rphi*Rtheta*Rpsidot;

x_d1 = -0.9;
y_d1 = 0.4;
z_d1 = 0.05;
x_d2 = 0.2;
y_d2 =-0.3;
z_d2 = 0.05;
x_d3 = 0.8;
y_d3 = 0.2;
z_d3 = 0.05;

pd1 = [x_d1;y_d1;z_d1];
pd2 = [x_d2;y_d2;z_d2];
pd3 = [x_d3;y_d3;z_d3];
pcom = [xcom;ycom;zcom];
pcomdot = [xcomdot;ycomdot;zcomdot];
p1 = pcom + R*pd1;
p1dot = pcomdot + Rdot*pd1;
p2 = pcom + R*pd2;
p2dot = pcomdot + Rdot*pd2;
p3 = pcom + R*pd3;
p3dot = pcomdot + Rdot*pd3;

ac_1 = x1 - dot([1 0 0],p1);
ac_1dot = x1dot - dot([1 0 0],p1dot);
ac_2 = x2 - dot([1 0 0],p2);
ac_2dot = x2dot - dot([1 0 0],p2dot);
ac_3 = x3 - dot([1 0 0],p3);
ac_3dot = x3dot - dot([1 0 0],p3dot);
bc_1 = y1 - dot([0 1 0],p1);
bc_1dot = y1dot - dot([0 1 0],p1dot);
bc_2 = y2 - dot([0 1 0],p2);
bc_2dot = y2dot - dot([0 1 0],p2dot);
bc_3 = y3 - dot([0 1 0],p3);
bc_3dot = y3dot - dot([0 1 0],p3dot);
cc_1 = z1 - dot([0 0 1],p1);
cc_1dot = z1dot - dot([0 0 1],p1dot);
cc_2 = z2 - dot([0 0 1],p2);
cc_2dot = z2dot - dot([0 0 1],p2dot);
cc_3 = z3 - dot([0 0 1],p3);
cc_3dot = z3dot - dot([0 0 1],p3dot);
acscc_1 = ac_1/cc_1;
acscc_1dot = (cc_1*ac_1dot-ac_1*cc_1dot)/(cc_1^2);
acscc_2 = ac_2/cc_2;
acscc_2dot = (cc_2*ac_2dot-ac_2*cc_2dot)/(cc_2^2);
acscc_3 = ac_3/cc_3;
acscc_3dot = (cc_3*ac_3dot-ac_3*cc_3dot)/(cc_3^2);
bcscc_1 = bc_1/cc_1;
bcscc_1dot = (cc_1*bc_1dot-bc_1*cc_1dot)/(cc_1^2);
bcscc_2 = bc_2/cc_2;
bcscc_2dot = (cc_2*bc_2dot-bc_2*cc_2dot)/(cc_2^2);
bcscc_3 = bc_3/cc_3;
bcscc_3dot = (cc_3*bc_3dot-bc_3*cc_3dot)/(cc_3^2);

u = p1-p2;
udot = Rdot*(pd1-pd2);
v = p1-p3;
vdot = Rdot*(pd1-pd3);

N = cross(u,v)';
Nx = N(1);
Ny = N(2);
Nz = N(3);
Nxdot = dot([1 0 0], cross(udot,v)+cross(u,vdot));
Nydot = dot([0 1 0], cross(udot,v)+cross(u,vdot));
Nzdot = dot([0 0 1], cross(udot,v)+cross(u,vdot));
boa = Ny/Nx;
boadot = (Nydot*Nx-Ny*Nxdot)/(Nx^2);
coa = Nz/Nx;
coadot = (Nzdot*Nx-Nz*Nxdot)/(Nx^2);
cob = Nz/Ny;
cobdot = (Nzdot*Ny-Nz*Nydot)/(Ny^2);

dx(:,1) = x1dot; %x1.
dx(:,2) = (1/m_ag)*(u_x1-F_1*tan(a_1)); %x1..
dx(:,3) = y1dot; %y1.
dx(:,4) = (1/m_ag)*(u_y1 - F_1*tan(b_1)); %y1..
dx(:,5) = z1dot; %z1.
dx(:,6) = (1/m_ag)*(u_z1 - F_1 - m_ag*g); %z1..
dx(:,7) = x2dot; %x2.
dx(:,8) = (1/m_ag)*(u_x2 - F_2*tan(a_2)); %x2..
dx(:,9) = y2dot; %y2.
dx(:,10) = (1/m_ag)*(u_y2 - F_2*tan(b_2)); %y2..
dx(:,11) = z2dot; %z2.
dx(:,12) = (1/m_ag)*(u_z2 - F_2 - m_ag*g); %z2..
dx(:,13) = x3dot; %x3.
dx(:,14) = (1/m_ag)*(u_x3 - F_3*tan(a_3)); %x3..
dx(:,15) = y3dot; %y3.
dx(:,16) = (1/m_ag)*(u_y3 - F_3*tan(b_3)); %y3..
dx(:,17) = z3dot; %z3.
dx(:,18) = (1/m_ag)*(u_z3 - F_3 - m_ag*g); %z3..
dx(:,19) =  xcomdot;%xcom.
dx(:,20) =  (1/m_pl)*(u_x1+u_x2+u_x3 - m_ag*(dx(:,2)+dx(:,8)+dx(:,14)));%xcom..
dx(:,21) =  ycomdot;%ycom.
dx(:,22) =  (1/m_pl)*(u_y1+u_y2+u_y3 - m_ag*(dx(:,4)+dx(:,10)+dx(:,16)));%ycom..
dx(:,23) =  zcomdot;%zcom.
dx(:,24) =  (1/m_pl)*(u_z1+u_z2+u_z3 - m_ag*(dx(:,6)+dx(:,12)+dx(:,18) - 3*g));%zcom..

%Problematic
dx(:,25) = g*(m_pl+2*m_ag)+m_ag*(dx(:,12)+dx(:,18))-(u_z2+u_z3);%F_1
dx(:,26) =  g*(m_pl+2*m_ag)+m_ag*(dx(:,6)+dx(:,18))-(u_z1+u_z3);%F_2
dx(:,27) =  g*(m_pl+2*m_ag)+m_ag*(dx(:,6)+dx(:,12))-(u_z1+u_z2);%F_3

dx(:,28) =  atan(Ny/Nx);%phi
dx(:,29) =  (1/(1+boa^2))*boadot;%phi.
dx(:,30) =  atan(Nz/Nx);%theta
dx(:,31) =  (1/(1+coa^2))*coadot;%theta.
dx(:,32) =  atan(Nz/Ny);%psi
dx(:,33) =  (1/(1+cob^2))*cobdot;%psi.

dx(:,34) = atan(acscc_1);% a_1
dx(:,35) = (1/(1+acscc_1^2))*acscc_1dot;% a_1dot
dx(:,36) = atan(bcscc_1);% b_1
dx(:,37) = (1/(1+bcscc_1^2))*bcscc_1dot;% b_1dot
dx(:,38) = atan(acscc_2);% a_2
dx(:,39) = (1/(1+acscc_2^2))*acscc_2dot;% a_2dot
dx(:,40) = atan(bcscc_2);% b_2
dx(:,41) = (1/(1+bcscc_2^2))*bcscc_2dot;% b_2dot
dx(:,42) = atan(acscc_3);% a_3
dx(:,43) = (1/(1+acscc_3^2))*acscc_3dot;% a_3dot
dx(:,44) = atan(bcscc_3);% b_3
dx(:,45) = (1/(1+bcscc_3^2))*bcscc_3dot;% b_3dot

%------------- END OF CODE --------------