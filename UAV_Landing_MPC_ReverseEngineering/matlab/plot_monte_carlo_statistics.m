function plot_monte_carlo_statistics(metrics, params)
%PLOT_MONTE_CARLO_STATISTICS Histograms matching the paper's evaluation.

fig = figure('Name', 'Touchdown statistics', 'Color', 'w');
tiledlayout(fig, 3, 2, 'TileSpacing', 'compact');

plot_hist(metrics.positionDeviation, 'position deviation [m]');
plot_hist(metrics.horizontalVelocityDeviation, 'horizontal velocity deviation [m/s]');
plot_hist(metrics.rollDeviation, 'roll deviation [deg]');
plot_hist(metrics.pitchDeviation, 'pitch deviation [deg]');
plot_hist(metrics.yawDeviation, 'yaw deviation [deg]');
plot_hist(metrics.touchdownTime, 'landing duration [s]');

saveas(fig, fullfile(params.paths.results, 'monte_carlo_statistics.png'));

fprintf('\nMonte Carlo summary:\n');
fprintf('  success rate: %.1f %%\n', 100*mean(metrics.success));
fprintf('  position deviation N(%.3f, %.3f^2)\n', mean(metrics.positionDeviation), std(metrics.positionDeviation));
fprintf('  horizontal velocity N(%.3f, %.3f^2)\n', mean(metrics.horizontalVelocityDeviation), std(metrics.horizontalVelocityDeviation));
fprintf('  roll deviation N(%.3f, %.3f^2) deg\n', mean(metrics.rollDeviation), std(metrics.rollDeviation));
fprintf('  pitch deviation N(%.3f, %.3f^2) deg\n', mean(metrics.pitchDeviation), std(metrics.pitchDeviation));
fprintf('  yaw deviation N(%.3f, %.3f^2) deg\n', mean(metrics.yawDeviation), std(metrics.yawDeviation));
end

function plot_hist(data, labelText)
nexttile;
histogram(data, 12, 'Normalization', 'pdf');
grid on; xlabel(labelText); ylabel('pdf');
title(sprintf('N(%.2f, %.2f^2)', mean(data), std(data)));
end

