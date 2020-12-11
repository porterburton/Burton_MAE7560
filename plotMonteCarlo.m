function [ hfigs ] = plotMonteCarlo(errors, traj_ref, traj, simpar)
%PLOTMONTECARLO_GPSINS creates hair plots of the estimation error for each
%state.  I left example code so you can see how I normally make these
%plots.
[n, ~, ~] = size(errors);
hfigs = [];
%% Plot estimation errors
ylabels = {'X_i Position Est Err (m)',...
    'Y_i Position Est Err (m)',...
    'Z_i Position Est Err (m)',...
    'X_i Velocity Est Err (m/s)',...
    'Y_i Velocity Est Err (m/s)',...
    'Z_i Velocity Est Err (m/s)',...
    'Range Bias Est Err (m)',...
    'X_i Gravity Anomoly Est Err',...
    'Y_i Gravity Anomoly Est Err',...
    'Z_i Gravity Anomoly Est Err',...
    'h Terrain Height Est Err',...
    'X_b Accl Bias Est Err (m/s^2)',...
    'Y_b Accl Bias Est Err (m/s^2)',...
    'Z_b Accl Bias Est Err (m/s^2)'};
for i=1:n
    
    hfigs(end + 1) = figure('Name',sprintf('est_err_%d',i)); %#ok<*AGROW>
    hold on;
    grid on;
    ensemble = squeeze(errors(i,:,:));
    filter_cov = squeeze(traj_ref.navCov(i,i,:));
    h_hair = stairs(traj_ref.time_nav, ensemble,'Color',[0.8 0.8 0.8]);
    h_filter_cov = stairs(traj_ref.time_nav, ...
        [3*sqrt(filter_cov) -3*sqrt(filter_cov)],'--r');
    legend([h_hair(1), h_filter_cov(1)],'MC run','EKF cov')
    xlabel('time(s)')
    ylabel(ylabels{i})
end

figure
xError = squeeze(errors(1,end,:));
yError = squeeze(errors(2,end,:));
zError = squeeze(errors(3,end,:));


plot(xError, zError, 'r*');
hold on
plot(0,0, 'bo', 'MarkerSize', 12);
xlabel('x (m)')
ylabel('z (m)')
legend('MonteCarlo Runs', 'Target Landing Site')
axis 'equal'
title('Landing Site Errors')
end