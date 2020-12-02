function [ P_dot ] = navCov_de( P, input )
%navCov_de computes the derivative of the nav state covariance
%
% Inputs:
%   Phat = nav state (mixed units)
%   input = input (mixed units)
%
% Outputs
%   Phat_dot = nav state derivative (mixed units)
%
% Example Usage
% [ Phat_dot ] = navCov_de( Phat, input )

% Author: Randy Christensen
% Date: 21-May-2019 10:40:10
% Reference: none
% Copyright 2019 Utah State University

%Unpack the inputs for clarity
xt = input.xt;
xhat = input.xhat;
simpar = input.simpar;
ytilde = input.ytilde;
v_perp = input.v_perp;

%Compute state dynamics matrix
F = calc_F(xhat, ytilde, simpar);

%Compute process noise coupling matrix
B = calc_B(xhat, simpar);

%Compute process noise PSD matrix Q
Q_a = 2*simpar.nav.params.Q_nongrav*eye(3);
Q_rbias = 2*simpar.nav.params.sig_rbias_ss^2/simpar.general.tau_r;
Q_abias = 2*simpar.nav.params.sig_abias_ss^2/simpar.general.tau_a*eye(3);
Q_g = 2*norm(v_perp)*simpar.nav.params.sig_grav_ss^2/simpar.general.d_g*eye(3);
Q_h = 2*norm(v_perp)*simpar.nav.params.sig_h_ss^2/simpar.general.d_h;

Q = blkdiag(Q_a,Q_rbias, Q_g, Q_h, Q_abias);

%Compute Phat_dot
P_dot = F*P+P*F'+B*Q*B';
end
