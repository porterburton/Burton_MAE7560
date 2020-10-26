function [ a ] = guidance(r, v, af, vf, rf, tgo, g, varargin)
%guidance computes the guidance acceleration
%
% Inputs:
%   r = position (m)
%   v = velocity (m)
%   af = desired final acceleration (m/s^2)
%   vf = desired final velocity (m/s)
%   rf = desired final position (m)
%   tgo = time-to-go (s)
%   g = gravity (m/s^2)
%
% Outputs
%   a = commanded acceleration (m/s^2)
%
% Example Usage
% [ a ] = guidance(r, v, af, vf, rf, tgo, g, varargin)

% Author: Randy Christensen
% Date: 28-Sep-2020 19:48:34
% Reference: 2018, Ping Lu, "Theory of Fractional-Polynomial Powered
% Descent Guidance", Eqn 34
% Copyright 2020 Utah State University

%% Handle varargin
if length(varargin) >= 1
    option = varargin{1};
else
    option = 'apollo';
end
%% Compute the guidance acceleration
if strcmp(option, 'e-guidance')
    if tgo > 0.001
        a = -2/tgo*(vf - v) + 6/tgo^2*(rf - r - v*tgo) - g;
    else
        a = zeros(3,1);
    end
else %apollo
    if tgo > 0.001  
        a = af - 6/tgo*(vf - v) + 12/tgo^2*(rf - r - v*tgo);
    else
        a = zeros(3,1);
    end
end
end
