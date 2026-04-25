function u_virtual = virtual_input(agents)

[u_VFG,~,~,~] = VFG(agents);
u_ACS = ACS(agents);
u_PF = PF(agents);

u_virtual = u_VFG + u_ACS + u_PF;