function xhatdot = navState_de(xhat,input)
%navState_de computes the derivative of the nav state
%
% Inputs:
%   xhat = nav state (mixed units)
%   input = input (mixed units)
%
% Outputs
%   xhatdot = nav state derivative (mixed units)
%
% Example Usage
% xhatdot = navState_de(xhat,input)

% Author: Randy Christensen
% Date: 21-May-2019 10:40:10
% Reference: none
% Copyright 2019 Utah State University

%% Unpack the inputs
simpar = input.simpar;
mu = simpar.general.mu;
T_i2b = calc_attitude(xhat, simpar);
T_b2i = T_i2b'; %Need body to inertial for velocity (in inertial)
omega_mi_m = [0;0;simpar.general.omega_moon];

r = xhat(simpar.states.ixf.pos);
v = xhat(simpar.states.ixf.vel);
br = xhat(simpar.states.ixf.br);
eps_gi = xhat(simpar.states.ixf.gbias);
h = xhat(simpar.states.ixf.h);
ba = xhat(simpar.states.ixf.ba);

v_perp = input.v_perp;
a_tilde = input.ytilde;
tau_r = simpar.general.tau_r;
tau_a = simpar.general.tau_a;
d_g = simpar.general.d_g;
d_h = simpar.general.d_h;
%% Compute individual elements of x_dot
rdot = v;
a_grav = -mu/norm(r)^3*r;
vdot = T_b2i*(a_tilde-ba)+a_grav+eps_gi;
bdot_r = -br/tau_r;

v_perp = v-cross(omega_mi_m,r)-dot(v, r/norm(r))*r/norm(r);
epsdot_gi = -norm(v_perp)/d_g*eps_gi;
hdot = -norm(v_perp)/d_h*h;
bdot_a = -ba/tau_a;

%% Assign to output
xhatdot = [rdot;vdot;bdot_r;epsdot_gi;hdot;bdot_a];
end
