function [ xhat ] = truth2nav( x_t )
%truth2nav maps the truth state vector to the navigation state vector
%
% Inputs:
%   x_t = truth state (mixed units)
%
% Outputs
%   xhat = navigation state (mixed units)
%
% Example Usage
% [ xhat ] = truth2nav( x )

% Author: Randy Christensen
% Date: 21-May-2019 14:17:45
% Reference: 
% Copyright 2019 Utah State University
[n, ~] = size(x_t);
assert(n == 22)
M = zeros(14,22);
M(1:6,1:6) = eye(6,6);
M(7:10,15:18) = eye(4,4);
M(11:14,19:22) = eye(4,4);
xhat = M*x_t;
end
