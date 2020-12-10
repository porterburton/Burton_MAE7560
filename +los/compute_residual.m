function [res] = compute_residual(xhat, z_tilde ,simpar, r_fi,Ti2b)
z_tildehat = los.pred_measurement(xhat, r_fi, simpar, Ti2b);
res = z_tilde-z_tildehat;
end

