function [ xhat ] = initialize_nav_state(simpar, xt)

xhat = truth2nav(xt);
%Inject Errors into the Position and velocity states 
%(all other states have been randomized already)
%TODO: Check that you are using the nav indices where you should.
xhat(simpar.states.ix.pos(1)) = xhat(simpar.states.ix.pos(1))+simpar.truth.ic.sig_rx*randn;
xhat(simpar.states.ix.pos(2)) = xhat(simpar.states.ix.pos(2))+simpar.truth.ic.sig_ry*randn;
xhat(simpar.states.ix.pos(3)) = xhat(simpar.states.ix.pos(3))+simpar.truth.ic.sig_rz*randn;

xhat(simpar.states.ix.vel(1)) = xhat(simpar.states.ix.vel(1))+simpar.truth.ic.sig_vx*randn;
xhat(simpar.states.ix.vel(2)) = xhat(simpar.states.ix.vel(2))+simpar.truth.ic.sig_vy*randn;
xhat(simpar.states.ix.vel(3)) = xhat(simpar.states.ix.vel(3))+simpar.truth.ic.sig_vz*randn;

xhat(simpar.states.ixf.parameter) = 0;
end
