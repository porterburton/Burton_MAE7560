function xdot = truthState_de( x, input)
%truthState_de computes the derivative of the truth state
%
% Inputs:
%   x = truth state (mixed units)
%   u = input (mixed units)
%
% Outputs
%   xdot = truth state derivative (mixed units)
%
% Example Usage
% xdot = truthState_de( x, input)

% Author: Randy Christensen
% Date: 21-May-2019 10:40:10
% Reference: none
% Copyright 2019 Utah State University

%% Unpack the inputs
simpar = input.simpar;
omega = simpar.general.omega_moon;
mu = simpar.general.mu; %m3/s2
w_a = input.w_a;
w_r = input.w_r;
w_d = input.w_d;
w_h = input.w_h;
w_accl = input.w_accl;
omega_mi_m = [0;0;omega]; %moon rotational velocity

a_thrust = input.a_t;
v_perp = input.v_perp;
tau_r = simpar.general.tau_r;
tau_a = simpar.general.tau_a;
d_g = simpar.general.d_g;
d_h = simpar.general.d_h;
%%
% 
% <latex>
% \begin{tabular}{|c|c|} \hline
% $n$ & $n!$ \\ \hline
% 1 & 1 \\
% 2 & 2 \\
% 3 & 6 \\ \hline
% \end{tabular}
% </latex>
% 
%% Unpack x 
r = x(simpar.states.ix.pos);
v = x(simpar.states.ix.vel);
q_im = x(simpar.states.ix.att);
q_bc = x(simpar.states.ix.cam_att);
br = x(simpar.states.ix.br);
eps_gi = x(simpar.states.ix.gbias);
h = x(simpar.states.ix.h);
ba = x(simpar.states.ix.ba);
%% Compute individual elements of x_dot
rdot = v;

a_grav = -mu/norm(r)^3*r;
vdot = a_grav+a_thrust+eps_gi+w_a;
qdot_im = 1/2*qmult([0;omega_mi_m],q_im);
qdot_bc = zeros(4,1);
bdot_r = -br/tau_r+w_r;
v_perp = v-cross(omega_mi_m,r)-dot(v, r/norm(r))*r/norm(r);
epsdot_gi = -norm(v_perp)/d_g*eps_gi+w_d;
hdot = -norm(v_perp)/d_h*h+w_h;
bdot_a = -ba/tau_a+w_accl;


%% Assign to output
xdot = [rdot;vdot;qdot_im;qdot_bc;bdot_r;epsdot_gi;hdot;bdot_a];
end