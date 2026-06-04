function plot_landing_results(result, params, tag)
%PLOT_LANDING_RESULTS Paper-style diagnostic plots.

if nargin < 3
    tag = 'trial';
end

t = result.log.t;
x = result.log.x;
usv = result.log.usv;
modeIdx = mode_to_index(result.log.mode);

fig = figure('Name', 'MPC-FAA landing reverse engineering', 'Color', 'w');
tiledlayout(fig, 4, 1, 'TileSpacing', 'compact');

nexttile;
plot(t, x(:,1), 'b', t, usv(:,1), 'b--', t, x(:,2), 'r', t, usv(:,2), 'r--', 'LineWidth', 1.2);
grid on; ylabel('x,y [m]');
legend('UAV x','USV x','UAV y','USV y', 'Location', 'best');

nexttile;
plot(t, x(:,3), 'k', t, usv(:,3), 'k--', 'LineWidth', 1.2);
grid on; ylabel('z [m]');
legend('UAV z','USV deck z', 'Location', 'best');

nexttile;
plot(t, rad2deg(x(:,4)-usv(:,4)), 'b', t, rad2deg(x(:,5)-usv(:,5)), 'r', ...
    t, rad2deg(x(:,6)-usv(:,6)), 'k', 'LineWidth', 1.2);
grid on; ylabel('attitude error [deg]');
legend('roll','pitch','yaw', 'Location', 'best');

nexttile;
stairs(t, modeIdx, 'LineWidth', 1.2);
grid on; ylabel('mode'); xlabel('time [s]');
yticks(1:7);
yticklabels({'GET_ALT','APPROACH','TRACK','STABLE','DESCENT','FLARE','LANDED'});

saveas(fig, fullfile(params.paths.results, [tag '_timeseries.png']));

fig2 = figure('Name', 'Top-view path', 'Color', 'w');
plot(usv(:,1), usv(:,2), 'k--', 'LineWidth', 1.5); hold on;
plot(x(:,1), x(:,2), 'b', 'LineWidth', 1.5);
plot(usv(end,1), usv(end,2), 'ko', 'MarkerFaceColor', 'k');
plot(x(end,1), x(end,2), 'bo', 'MarkerFaceColor', 'b');
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
legend('USV','UAV','USV touchdown','UAV touchdown', 'Location', 'best');
saveas(fig2, fullfile(params.paths.results, [tag '_top_view.png']));
end

function idx = mode_to_index(modes)
names = {'GET_ALTITUDE','APPROACH','TRACKING','TRACKING_STABLE','DESCENT','FLARE','LANDED'};
idx = zeros(numel(modes),1);
for i = 1:numel(modes)
    idx(i) = find(strcmp(names, modes{i}), 1, 'first');
end
end

