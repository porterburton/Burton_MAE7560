function[T_i2b] = calc_attitude(x,simpar)

% %iniertial z axis
uz_ii = [0;0;1];
%Velocity vector
v_i = x(simpar.states.ix.vel);
r_i = x(simpar.states.ix.pos);
u_v_i = v_i/norm(v_i);
%Velocity will define the y axis assuming we always point the spacecrats thruster in velocity direction
u_yb_i = -u_v_i;
%Define the body z axis to be orthogonal to the orbital plane
assert(norm(cross(r_i,v_i)) > 1e-6,'Velocity is parallel or anti parallel with position!')
% fprintf('norm(cross(r_i,v_i)) = %d\n',norm(cross(r_i,v_i)));
u_h_i = -cross(r_i,v_i)/norm(cross(r_i,v_i));
u_zb_i = u_h_i;
%Define the body x axis via right hand rule
u_xb_i = cross(u_yb_i,u_zb_i);
%Compile the DCM from interial to body
T_i2b = [u_xb_i';u_yb_i';u_zb_i'];

%Cross to find x axis
% th = 270*pi/180;
% rot_z = [cos(th), -sin(th),0;
%         sin(th), cos(th), 0;
%         0, 0, 1];
    
% ux_bi = cross(uy_bi, uz_ii);
% ux_bi = ux_bi/norm(ux_bi);
% T_i2b = [ux_bi';uy_bi';uz_ii'];

% v_bi_i = x(simpar.states.ix.vel);
% r_bi_i = x(simpar.states.ix.pos);
% u_yb_i = -v_bi_i/norm(v_bi_i);
% u_zb_i = cross(r_bi_i,v_bi_i)/norm(cross(r_bi_i,v_bi_i));
% u_xb_i = cross(u_yb_i,u_zb_i);
% T_i2b = [u_xb_i';u_yb_i';u_zb_i'];

assert(all(all(abs(T_i2b*T_i2b' - eye(3))<= 1e-12)),'T_i2b not orthonormal')
end