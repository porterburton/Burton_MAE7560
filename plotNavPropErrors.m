function h_figs = plotNavPropErrors(traj)
% In this function, you should plot everything you want to see for a
% single simulation when there are no errors present.  I like to see things
% like the 3D trajectory, altitude, angular rates, etc.  Two sets of plots
% that are required are the estimation errors for every state and the
% residuals for any measurements.  I left some of my code for a lunar
% lander application, so you can see how I do it.  Feel free to use or
% remove whatever applies to your problem.
%% Prelims
h_figs = [];
simpar = traj.simpar;
m2km = 1/1000;
%% Plot trajectory
h_figs(end+1) = figure;
htraj = plot3(traj.truthState(simpar.states.ix.pos(1),:)'*m2km,...
    traj.truthState(simpar.states.ix.pos(2),:)'*m2km,...
    traj.truthState(simpar.states.ix.pos(3),:)'*m2km,...
    'LineWidth',2);
hold all
hstart = scatter3(traj.truthState(simpar.states.ix.pos(1),1)'*m2km,...
    traj.truthState(simpar.states.ix.pos(2),1)'*m2km,...
    traj.truthState(simpar.states.ix.pos(3),1)'*m2km,...
    'filled','g');
hstop = scatter3(traj.truthState(simpar.states.ix.pos(1),end)'*m2km,...
    traj.truthState(simpar.states.ix.pos(2),end)'*m2km,...
    traj.truthState(simpar.states.ix.pos(3),end)'*m2km,...
    'filled','r');
xlabel('$x_i\left(km\right)$','Interpreter','latex')
ylabel('$y_i\left(km\right)$','Interpreter','latex')
zlabel('$z_i\left(km\right)$','Interpreter','latex')
grid on;
axis equal;
[X,Y,Z] = sphere;
R = simpar.general.r_moon/1000;
hmoon = surf(X*R,Y*R,Z*R,'FaceAlpha',0.2,...
    'FaceColor','k','EdgeAlpha',0.3);
legend([htraj, hstart, hstop, hmoon],...
    'trajectory','start','stop','moon','Interpreter','latex')
%% Plot 2D Position
h_figs(end+1) = figure;
plot(traj.truthState(simpar.states.ix.pos(1),:)'*m2km, traj.truthState(simpar.states.ix.pos(2),:)'*m2km,'LineWidth',2);
xlabel('$x_i\left(km\right)$','Interpreter','latex')
ylabel('$y_i\left(km\right)$','Interpreter','latex')
grid on;


%% Plot Velocity
h_figs(end+1) = figure;
plot(traj.time_nav, traj.truthState(simpar.states.ix.vel,:)'*m2km,'LineWidth',2);
xlabel('t')
ylabel('Velocity (km/s)')
legend('Vx','Vy','Vz')
grid on;
%% Plot Position
h_figs(end+1) = figure;
plot(traj.time_nav, traj.truthState(simpar.states.ix.pos,:)'*m2km,'LineWidth',2);
xlabel('t')
ylabel('Positon(km)')
legend('Rx','Ry','Rz')
grid on;
%% Plot A_trust
h_figs(end+1) = figure;
plot(traj.time_nav, traj.a_thrust'*m2km,'LineWidth',2);
xlabel('t')
ylabel('Thrust Acceleration (km/s^2)')
legend('Tx','Ty','Tz')
grid on;
%% Plot Altitude vs. time
r_mag = sqrt(traj.truthState(simpar.states.ix.pos(1),:).^2 ...
    + traj.truthState(simpar.states.ix.pos(2),:).^2 ...
    + traj.truthState(simpar.states.ix.pos(3),:).^2)*m2km;
alt = r_mag - simpar.general.r_moon*m2km;
h_figs(end+1) = figure;
stairs(traj.time_nav,alt);
% title('Altitude vs. time');
xlabel('time$\left(s\right)$','Interpreter','latex');
ylabel('altitude$\left(km\right)$','Interpreter','latex');
grid on;
%% Plot attitude vs. time
% nsamp = length(traj.time_nav);
% q_lvlh2b = zeros(4,nsamp);
% q_lvlh2c = zeros(4,nsamp);
% q_b2c = traj.q_b2c_nominal;
% for i=1:nsamp
%     q_i2b = traj.truthState(simpar.states.ix.att,i);
%     q_i2lvlh = tmat2q(inertial2lvlh(traj.truthState(simpar.states.ix.pos,i),...
%         traj.truthState(simpar.states.ix.vel,i)));
%     q_lvlh2b(:,i) = qmult(q_i2b,qConjugate(q_i2lvlh));
%     q_lvlh2c(:,i) = qmult(q_b2c,q_lvlh2b(:,i));
% end
% h_figs(end+1) = figure;
% stairs(traj.time_nav,q_lvlh2c');
% xlabel('time$\left(s\right)$','Interpreter','latex');
% ylabel('$q^{lvlh}_{c}$','Interpreter','latex')
% legend('q_0','q_i','q_j','q_k')
% grid on;
%% Plot camera angle of incidence
% h_figs(end+1) = figure;
% stairs(traj.time_nav, traj.angle_of_incidence_deg);
% xlabel('time$\left(s\right)$','Interpreter','latex');
% ylabel('$\gamma\left(deg\right)$','Interpreter','latex');
% grid on;
%% LOS Residuals
h_figs(end+1) = figure;
stairs(traj.time_kalman,traj.navRes.los');
xlabel('Time(s)')
ylabel('LOS residuals')
legend('l_{x}/l_{z}','l_{y}/l_{z}')
grid on;
%% Calculate estimation errors
dele = calcErrors(traj.navState, traj.truthState, simpar);
%% Plot position estimation error
h_figs(end+1) = figure;
stairs(traj.time_nav, dele(simpar.states.ixfe.pos,:)');
title('Position Error');
xlabel('time(s)');
ylabel('m');
legend('x_i','y_i','z_i')
grid on;
%% Plot velocity error
h_figs(end+1) = figure;
stairs(traj.time_nav, dele(simpar.states.ixfe.vel,:)');
title('Velocity Error');
xlabel('time(s)');
ylabel('m/s');
legend('x_i','y_i','z_i')
grid on;
%% Plot Range Bias Eroor
h_figs(end+1) = figure;
stairs(traj.time_nav, dele(simpar.states.ixfe.br,:)');
title('Range Bias Error');
xlabel('time(s)');
ylabel('m');
grid on;
%% Plot GravBias error
h_figs(end+1) = figure;
stairs(traj.time_nav, dele(simpar.states.ixfe.gbias,:)');
title('Gravity Bias Error');
xlabel('time(s)');
ylabel('m/s^2');
legend('x','y','z')
grid on;
%% Plot Terrain Height Error
h_figs(end+1) = figure;
stairs(traj.time_nav, dele(simpar.states.ixfe.h,:)');
title('Terrain Height Error');
xlabel('time(s)');
ylabel('m');
grid on;
%% Plot AcclBias error
h_figs(end+1) = figure;
stairs(traj.time_nav, dele(simpar.states.ixfe.ba,:)');
title('Accelerometer Bias Error');
xlabel('time(s)');
ylabel('mg');
legend('x','y','z')
grid on;
end