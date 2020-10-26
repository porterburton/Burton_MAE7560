function [ x, theta, dl, intersect ] = line_sphere_intersection( l, o, c, r )
%line_sphere_intersection intersects a line with a sphere.
%
% Inputs:
%   l = 3x1 unit vector in direction of line (unitless)
%   o = 3x1 position of origin of line (m)
%   c = 3x1 position of center of sphere (m)
%   r = radius of sphere (m)
%
% Outputs
%   x = 3x1 position of line/sphere intersection (m)
%   theta = angle of incidence of line with sphere (rad)
%   dl = 3x1 vector from line origin to line/sphere interesection (m)
%   intersect = description (units)
%
% Example Usage
% [ x, theta, intersect ] = line_sphere_intersection( l, o, c, r )

% Author: Randy Christensen
% Date: 16-Mar-2020 16:24:07
% Reference: https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
% Copyright 2020 Utah State University

%Normalize the pointing vector
l = l/norm(l);
term_sqrd = (l'*(o - c))^2 - ((o - c)'*(o - c) - r^2);
if (term_sqrd < 0)
    %No intersection
    x = nan(3,1);
    intersect = false;
    theta = nan;
    dl = nan;
elseif (term_sqrd == 0)
    %Only one intersection (i.e. it grazes the surface)
    d = -(l'*(o-c)) + sqrt(term_sqrd);
    x = o + d*l;
    intersect = true;
    %Compute the angle of incidence assuming a sphere
    dl = x - o;
    u_dl = -dl/norm(dl);
    u_x = x/norm(x);
    theta = acos(dot(u_dl,u_x));
else
    %Two intersections, where the closest point is chosen
    d = [-(l'*(o-c)) + sqrt(term_sqrd);
        -(l'*(o-c)) - sqrt(term_sqrd)];
    if any(d<0)
        x = nan;
        intersect = false;
        theta = nan;
        dl = nan;
    else
        x = o + min(d)*l;
        intersect = true;
        %Compute the angle of incidence assuming a sphere
        dl = x - o;
        u_dl = -dl/norm(dl);
        u_x = x/norm(x);
        theta = acos(dot(u_dl,u_x));
        assert(abs(norm(x-c)^2 - r^2)>=0,'Solution not valid')
    end
end
end
