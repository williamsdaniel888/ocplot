
function dx = SingleAgentQUAVPlant(x,u,p,t,vdat)
% SingleAgentQUAVPlant - Plant model for Adigator
%
% Single Agent Relocation Problem
% Copyright 2019
%
% Adapted from the Supersonic Aircraft Minimum Fuel Climb example for
% ICLOCS Version 2 (2018).
% The contribution of Yuanbo Nie, Omar Faqir, and Eric Kerrigan for their
% work on ICLOCS Version 2 (2018) is kindly acknowledged.
% Department of Electrical and Electronic Engineering,
% Imperial College London, UK
%--------------------------------------------------------

m_ag=0.81;
gc=9.81;

x2 = x(:,2);
x4 = x(:,4);
x6 = x(:,6);
u1 = u(:,1);
u2 = u(:,2);
u3 = u(:,3);

dx(:,1) = x2;
dx(:,2) = 1/m_ag*(u1);
dx(:,3) = x4;
dx(:,4) = 1/m_ag*(u2);
dx(:,5) = x6;
dx(:,6) = 1/m_ag*(u3-m_ag*gc);

%------------- END OF CODE --------------