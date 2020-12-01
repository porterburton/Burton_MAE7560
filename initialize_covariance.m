function [P0] = initialize_covariance(simpar)
%initialize_covariance computes the initial covariance matrix
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
% [ output_args ] = initialize_covariance( input_args )

P0 = zeros(simpar.states.nxfe, simpar.states.nxfe);
fnames = fieldnames(simpar.nav.ic);
for i=1:length(fnames)
    P0(i,i) = simpar.nav.ic.(fnames{i})^2;
end

end
