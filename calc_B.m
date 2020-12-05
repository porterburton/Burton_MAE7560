function [ B ] = calc_B(xhat, simpar )
%calc_G Calculates the process noise dynamic coupling matrix
%
% Inputs:
%   xhat = state vector
%   simpar= simulation parameters
%
% Outputs
%   G = process noise dynamic coupling matrix
%
% Example Usage
% [ B ] = calc_B( xhat, simpar )

% Author: Randy Christensen
% Date: 13-May-2020
% Reference: None
% Copyright 2020 Utah State University

%% Preliminary Calcs
Ti2b = calc_attitude(xhat, simpar);
Tb2i = Ti2b';
n = simpar.states.nxfe;
B = zeros(n,11); %Magic Number (number of driving noise values)
%% Compute B
B(simpar.states.ix.vel,simpar.states.ix.pos) = -Tb2i;
B(7:14,4:11) = eye(8); %more magic numbers
end
