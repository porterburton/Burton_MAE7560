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
%Position and Velocity will always be set to the Inital Condition
%Randomize all other truth states
xt(simpar.states.ix.br) = simpar.truth.ic.sig_br*randn;
xt(simpar.states.ix.gbias(1)) = simpar.truth.ic.sig_epsx*randn;
xt(simpar.states.ix.gbias(2)) = simpar.truth.ic.sig_epsy*randn;
xt(simpar.states.ix.gbias(3)) = simpar.truth.ic.sig_epsz*randn;
xt(simpar.states.ix.h) = simpar.truth.ic.sig_h*randn;
xt(simpar.states.ix.ba(1)) = simpar.truth.ic.sig_acclx*randn;
xt(simpar.states.ix.ba(2)) = simpar.truth.ic.sig_accly*randn;
xt(simpar.states.ix.ba(3)) = simpar.truth.ic.sig_acclz*randn;


end
