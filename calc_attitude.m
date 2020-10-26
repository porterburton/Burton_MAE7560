function[T_i2b] = calc_attitude(x,simpar)

%iniertial z axis
uz_ii = [0;0;1];
%Velocity vector 
u_v_u = x(simpar.states.ix.vel);
u_v_u = u_v_u/norm(u_v_u);
%Velocity will define the y axis assuming we always point the spacecrats thruster in velocity direction 
uy_bi = -u_v_u;
%Cross to find x axis
ux_bi = cross(uy_bi, uz_ii);
ux_bi = ux_bi/norm(ux_bi);
T_i2b = [ux_bi';uy_bi';uz_ii'];
end