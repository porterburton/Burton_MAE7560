function [ P_plus ] = update_covariance(K, H, P_minus, R, simpar)


%Don't forget to perform numerical checking and conditioning of covariance
%matrix
I = eye(simpar.states.nxfe);
G = eye(2);
P_plus = (I-K*H)*P_minus*(I-K*H)'+K*G*R*G'*K';
end
