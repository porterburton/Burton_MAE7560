function [res] = compute_residual(xhat, z_tilde ,simpar, r_fi)
z_tildehat = los.pred_measurement(xhat, r_fi, simpar);
res = z_tilde-z_tildehat;
end

