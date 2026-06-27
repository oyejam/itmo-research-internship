clear; clc; close all;
addpath(fileparts(mfilename('fullpath')));

params = initialize_paper_parameters();
nRuns = 100;

metrics = struct('positionDeviation', zeros(nRuns,1), ...
    'horizontalVelocityDeviation', zeros(nRuns,1), ...
    'rollDeviation', zeros(nRuns,1), ...
    'pitchDeviation', zeros(nRuns,1), ...
    'yawDeviation', zeros(nRuns,1), ...
    'touchdownTime', zeros(nRuns,1), ...
    'success', false(nRuns,1));

for i = 1:nRuns
    result = simulate_landing_trial(params, i, false);
    metrics.positionDeviation(i) = result.metrics.positionDeviation;
    metrics.horizontalVelocityDeviation(i) = result.metrics.horizontalVelocityDeviation;
    metrics.rollDeviation(i) = rad2deg(result.metrics.rollDeviation);
    metrics.pitchDeviation(i) = rad2deg(result.metrics.pitchDeviation);
    metrics.yawDeviation(i) = rad2deg(result.metrics.yawDeviation);
    metrics.touchdownTime(i) = result.touchdownTime;
    metrics.success(i) = result.success;
    fprintf('Run %3d/%3d: success=%d pos=%.3f m time=%.2f s\n', ...
        i, nRuns, result.success, result.metrics.positionDeviation, result.touchdownTime);
end

save(fullfile(params.paths.results, 'monte_carlo_metrics.mat'), 'metrics', 'params');
plot_monte_carlo_statistics(metrics, params);

