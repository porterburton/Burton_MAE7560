function [ Fhat ] = calc_F( xhat, Ti2b, simpar )
%calc_F computes the dynamics coupling matrix
%
% Inputs:  
%   xhat = state vector
%   ytilde = continuous measurements
%   simpar = simulation parameters
%
% Outputs
%   Fhat = state dynamics matrix
%
% Example Usage
% [ Fhat ] = calc_F( xhat, ytilde, simpar )

% Author: Randy Christensen
% Date: 13-May-2020
% Reference: None
% Copyright 2020 Utah State University
%% Get parameters
tau_r = simpar.general.tau_r;
tau_a = simpar.general.tau_a;
d_g = simpar.general.d_g;
d_h = simpar.general.d_h;
mu = simpar.general.mu;
omega_mi_m = [0;0;simpar.general.omega_moon];

rhat = xhat(simpar.states.ixfe.pos);
vhat = xhat(simpar.states.ixfe.vel);
%% Preliminary calulations for the jacobian
I = eye(3,3);
ir = xhat(simpar.states.ixfe.pos)/norm(rhat);
Fv = -mu/norm(rhat)^3*(I-3*(ir*ir'));

Fbr = -1/tau_r;

v_perp = vhat-cross(omega_mi_m,rhat)-dot(vhat, rhat/norm(rhat))*rhat/norm(rhat);
Fgbias = -norm(v_perp)/d_g*I;

Fh = -norm(v_perp)/d_h;

Fba = -1/tau_a*I;
%% Compute Fhat
Fhat = zeros(simpar.states.nxfe,simpar.states.nxfe);
Fhat(simpar.states.ixfe.pos, simpar.states.ixfe.vel) = I; 
Fhat(simpar.states.ixfe.vel, simpar.states.ixfe.pos) = Fv; 
Fhat(simpar.states.ixfe.vel, simpar.states.ixfe.gbias) = I;
Fhat(simpar.states.ixfe.vel, simpar.states.ixfe.ba) = -Ti2b;
Fhat(simpar.states.ixfe.br, simpar.states.ixfe.br) = Fbr;
Fhat(simpar.states.ixfe.gbias, simpar.states.ixfe.gbias) = Fgbias;
Fhat(simpar.states.ixfe.h, simpar.states.ixfe.h) = Fh;
Fhat(simpar.states.ixfe.ba, simpar.states.ixfe.ba) = Fba;
end