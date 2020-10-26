function [ r_fks_i, r_fki_i, intersect, theta ] = gen_landmark(r_ci_i, ...
    r_sc_c, qi2c, R_moon, location, varargin)
%gen_landmark generates a landmark directly beneath the spacecraft (location
%= 'below') or in the center of the FOV (location = 'center')
%
% Inputs:
%   r_ci_i = position of spacecraft (m)
%   r_sc_c = sensor lever arm (m) = 0
%   qi2c = quaternion from LCI to chassis frame (q_ib)
%   R_moon = radius of moon (m)
%   location = specified location of landmark ('below' or 'center')
%   qc2s = quaternion from the chassis frame to the sensor frame
%   (optional)=qbc
%
% Outputs
%   r_fki_i = location of the landmark wrt LCI origin (m)
%   intersect = flag to indicate succesful placement of landmark
%   r_fks_i = slant range in LCI
%
% Example Usage
% [ r_fki_i, intersect ] = gen_landmark(Nominal_Traj, Sim_Param, location, <qc2s>)

% Author: Randy Christensen
% Date: 13-Sep-2019 19:21:31
% Reference: https://en.wikipedia.org/wiki/Line-sphere_intersection
% Copyright 2019 Utah State University

%TODO: Verify that the angle of incidence handles the case where the
%sensor is pointing away from the surface

T_c2i = q2tmat(qi2c)';
r_si_i = r_ci_i + T_c2i * r_sc_c;
if strcmp(location,'below')
    %Generates a target directly beneath the spacecraft
    u_rsi_i = r_si_i/norm(r_si_i);
    r_fki_i = R_moon * u_rsi_i;
    intersect = true;
    %Compute the angle of incidence assuming a sphere
    r_fks_i = r_fki_i - r_si_i;
    theta = acos(dot(u_rsi_i,-r_fks_i/norm(r_fks_i)));
elseif strcmp(location,'center')
    %Prelims
    assert(length(varargin)>=1,...
        'Must specify the quaternion from the chassis frame to the sensor frame')
    %Generates a target at the center of the FOV of the sensor
    qc2s = varargin{1};
    Ti2c = q2tmat(qi2c);
    Tc2s = q2tmat(qc2s);
    Ti2s = Tc2s*Ti2c;
    c = [0 0 0]';  %center of sphere
    r = R_moon;  %radius of sphere
    o = r_si_i;  %origin of FOV
    l = Ti2s(3,:)';  %direction of center of FOV
    [ r_fki_i, theta, r_fks_i, intersect ] = ...
        line_sphere_intersection( l, o, c, r );
elseif strcmp(location,'axis')
    assert(length(varargin)>=1,...
        'Must specify axis along which to intersect the moon sphere')
    %Generates a target along the specified axis
    c = [0 0 0]';  %center of sphere
    r = R_moon;  %radius of sphere
    o = r_si_i;  %origin of FOV
    l = varargin{1};  %direction of axis
    assert(abs(norm(l) - 1) < 1e-14,'Axis must be a unit vector')
    [ r_fki_i, theta, r_fks_i, intersect ] = ...
        line_sphere_intersection( l, o, c, r );
else
    assert(false,'Must specify valid target location')
end
end