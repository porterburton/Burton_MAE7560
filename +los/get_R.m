function [R] = get_R(x, H, simpar)

sig_los = simpar.nav.params.sig_los;

R = diag([sig_los^2, sig_los^2]);
end