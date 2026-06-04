clear; clc; close all;

addpath(fileparts(mfilename('fullpath')));

params = initialize_paper_parameters();
fprintf('Running MPC-FAA UAV landing simulation based on Prochazka et al. 2024...\n');

result = simulate_landing_trial(params, 1, true);
plot_landing_results(result, params, 'single_trial');

save(fullfile(params.paths.results, 'single_trial_result.mat'), 'result', 'params');

fprintf('\nTouchdown summary:\n');
fprintf('  success:              %d\n', result.success);
fprintf('  touchdown time:       %.2f s\n', result.touchdownTime);
fprintf('  position deviation:   %.3f m\n', result.metrics.positionDeviation);
fprintf('  horizontal velocity:  %.3f m/s\n', result.metrics.horizontalVelocityDeviation);
fprintf('  roll deviation:       %.3f deg\n', rad2deg(result.metrics.rollDeviation));
fprintf('  pitch deviation:      %.3f deg\n', rad2deg(result.metrics.pitchDeviation));
fprintf('  yaw deviation:        %.3f deg\n', rad2deg(result.metrics.yawDeviation));

