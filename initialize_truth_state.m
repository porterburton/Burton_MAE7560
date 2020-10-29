function [ xt ] = initialize_truth_state(simpar)
%initialize_truth_state initialize the truth state vector consistent with
%the initial covariance matrix
%
% Inputs:
%   Input1 = description (units)
%   Input2 = description (units)
%
% Outputs
%   Output1 = description (units)
%   Output2 = description (units)
%
% Example Usage
% [ output_args ] = initialize_truth_state( input_args )

% Author: 
% Date: 31-Aug-2020 15:46:59
% Reference: 
% Copyright 2020 Utah State University

% In the initialization of the truth and navigation states, you need only
% ensure that the estimation error is consistent with the initial
% covariance matrix.  One realistic way to do this is to set the true 
% vehicle states to the same thing every time, and randomize any sensor 
% parameters.
fnames = fieldnames(simpar.general.ic);
xt = zeros(length(fnames),1);
%use inital conditions
for i=1:length(fnames)
    xt(i) = simpar.general.ic.(fnames{i});
end
xt(simpar.states.ix.cam_att) = simpar.general.q_b2c_nominal;
disp(xt)
end
