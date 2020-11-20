function [output] = validate_linearization(x, xhat, simpar)
%% Inject Error
fnames = fieldnames(simpar.errorInjection);
dele_injected = zeros(numel(fnames),1);
for j=1:length(fnames)
    dele_injected(j) = simpar.errorInjection.(fnames{j});
end
xhat = injectErrors(truth2nav(x), dele_injected, simpar);
%% Calculate residual
[z_tilde, r_fi] = los.synth_measurement(x,simpar);
delz_nl = los.compute_residual(xhat, z_tilde, simpar, r_fi);
H = los.get_H(x, xhat, simpar);
delz_l = H*dele_injected;
%% Compare linear and nonlinear residuals
measLinTable.delz_nl = delz_nl;
measLinTable.delz_l = delz_l;
measLinTable.difference = delz_nl - delz_l;
disp(struct2table(measLinTable));
end

