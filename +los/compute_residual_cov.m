function [resCov] = compute_residual_cov(H, P, R)

G = eye(2);
resCov = H*P*H'+G*R*G';
end