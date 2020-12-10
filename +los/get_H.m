function [H] = get_H(xt, xhat, simpar, Ti2b)

%prelims
qb2c = simpar.general.q_b2c_nominal;
Tb2c = q2tmat(qb2c);
qi2b = tmat2q(Ti2b);

r_bi = xt(simpar.states.ix.pos);
r_bi_hat = xhat(simpar.states.ixfe.pos);
R_moon = simpar.general.r_moon;
[~, r_fi, intersect, ~] = gen_landmark(r_bi, zeros(3,1), qi2b, R_moon, 'center',...
    qb2c);

%Cacluatle H
lhat = Tb2c*Ti2b*(r_fi-r_bi_hat);
lx_hat = lhat(1); 
ly_hat = lhat(2);
lz_hat = lhat(3);

dh_dl = [1/lz_hat, 0, -lx_hat/lz_hat^2;
         0, 1/lz_hat, -ly_hat/lz_hat^2];

dl_dx = zeros(3,14);
dl_dx(1:3,1:3) = -Tb2c*Ti2b*eye(3,3);

H=dh_dl*dl_dx;

end

