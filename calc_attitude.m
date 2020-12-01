function[T_i2b] = calc_attitude(x,simpar)

% %iniertial z axis
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

% v_bi_i = x(simpar.states.ix.vel);
% r_bi_i = x(simpar.states.ix.pos);
% u_yb_i = -v_bi_i/norm(v_bi_i);
% u_zb_i = cross(r_bi_i,v_bi_i)/norm(cross(r_bi_i,v_bi_i));
% u_xb_i = cross(u_yb_i,u_zb_i);
% T_i2b = [u_xb_i';u_yb_i';u_zb_i'];

assert(all(all(abs(T_i2b*T_i2b' - eye(3))<= 1e-12)),'T_i2b not orthonormal')
end