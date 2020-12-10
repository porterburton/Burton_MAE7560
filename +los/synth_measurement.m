function[z_tilde,r_fi] = synth_measurement(x,simpar,Ti2b)
r_bi = x(simpar.states.ix.pos);
R_moon = simpar.general.r_moon;

%Inertial to body
qi2b = tmat2q(Ti2b);
%Inertial to moon (NOT NEEDED FOR NOW)
qi2m = x(simpar.states.ix.att);
%Body to camera frame
qb2c = x(simpar.states.ix.cam_att);
Tb2c = q2tmat(qb2c);

%Find postion to feaure in inertial frame   
[~, r_fi, intersect, ~] = gen_landmark(r_bi, zeros(3,1), qi2b, R_moon, 'center',...
    qb2c);
assert(intersect,'Camera did not intersect the moon!');
%r_fi= [simpar.general.rx_star_tf; simpar.general.ry_star_tf; simpar.general.rz_star_tf];
% r_fi = [200e3;1730e3;100e3];
%Determine lc 
lc = Tb2c*Ti2b*(r_fi-r_bi);
lx = lc(1);
ly = lc(2);
lz = lc(3);


%Camera Noise
nu_c = simpar.truth.params.sig_los*randn(2,1);
z_tilde = [lx/lz;ly/lz]+nu_c;
end