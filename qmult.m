function [ q1q2 ] = qmult( q1, q2 )
%qmult performs non-hamiltonian quaternion multiplications with the scalar
%(or real) term in the first component
%
% Inputs:
%   q1 = first quaternion
%   q2 = second quaternion
%
% Outputs
%   q1q2 = quaternion multiplication
%
% Example Usage
% [ q1q2 ] = qmult( q1, q2 )

% Author: Randy Christensen
% Date: 13-May-2020 08:52:26
% Reference: Rotations, Transformations, Left Quaternions, Right
% Quaternions?, Renato Zanetti, 2019, equation 35
% Copyright 2020 Utah State University

%% Validate inputs
[m_q1,n_q1] = size(q1);
[m_q2,n_q2] = size(q2);
assert(m_q1 == 4,'Incorrect size of q1')
assert(m_q2 == 4,'Incorrect size of q2')
assert(n_q1 == n_q2,'Size of inputs must match')
%% Compute transformation matrix
q1_scalar = q1(1,:);
q2_scalar = q2(1,:);
q1_vec = q1(2:4,:);
q2_vec = q2(2:4,:);
q1q2_vec = q1_scalar.*q2_vec + q2_scalar.*q1_vec - cross(q1_vec,q2_vec);
q1q2_scalar = q1_scalar.*q2_scalar - dot(q1_vec,q2_vec);
q1q2 = [q1q2_scalar; q1q2_vec];
end