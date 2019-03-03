dxdt = newtonian(m_ag,m_pl,g,u_x1,u_y1,u_z1,u_x2,u_y2,u_z2,u_x3,u_y3,u_z3,a_1,b_1,a_2,b_2,a_3,b_3,F_1,F_2,F_3)

dxdt(1) = dxdt(2); %x1.
dxdt(2) = (1/m_ag)*(u_x1 - F_1*tan(a_1)); %x1..
dxdt(3) = dxdt(4); %y1.
dxdt(4) = (1/m_ag)*(u_y1 - F_1*tan(b_1)); %y1..
dxdt(5) = dxdt(6); %z1.
dxdt(6) = (1/m_ag)*(u_z1 - F_1 -m_ag*g); %z1..
dxdt(7) = dxdt(8); %x2.
dxdt(8) = (1/m_ag)*(u_x2 - F_1*tan(a_2)); %x2..
dxdt(9) = dxdt(10); %y2.
dxdt(10) = (1/m_ag)*(u_y2 - F_1*tan(b_2)); %y2..
dxdt(11) = dxdt(12); %z2.
dxdt(12) = (1/m_ag)*(u_z2 - F_2 -m_ag*g); %z2..
dxdt(13) = dxdt(14); %x3.
dxdt(14) = (1/m_ag)*(u_x3 - F_1*tan(a_3)); %x3..
dxdt(15) = dxdt(16); %y3.
dxdt(16) = (1/m_ag)*(u_y3 - F_1*tan(b_3)); %y3..
dxdt(17) = dxdt(18); %z3.
dxdt(18) = (1/m_ag)*(u_z3 - F_3 -m_ag*g); %z3..
dxdt(19) = dxdt(20); %xcom.
dxdt(20) = (1/m_pl)*(u_x1 + u_x2 + u_x3 - m_ag*(dxdt(2) + dxdt(8) + dxdt(14)) ); %xcom..
dxdt(21) = dxdt(22); %ycom.
dxdt(22) = (1/m_pl)*(u_y1 + u_y2 + u_y3 - m_ag*(dxdt(4) + dxdt(10) + dxdt(16)) ); %ycom..
dxdt(23) = dxdt(24); %zcom.
dxdt(24) = (1/m_pl)*(u_z1 + u_z2 + u_z3 - m_ag*(dxdt(6) + dxdt(12) + dxdt(18)) - 3*m_ag); %zcom..

end