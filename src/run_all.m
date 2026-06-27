function run_all()
%RUN_ALL  One-command driver for the UAV-on-boat MPC-FAA reproduction.
%
%   Reproduces the experiments of Prochazka et al. (2024), "Model predictive
%   control-based trajectory generation for agile landing of unmanned aerial
%   vehicle on a moving boat," Ocean Engineering 318:119164.
%
%   Usage (from MATLAB R2021a or newer, Optimization Toolbox recommended):
%       >> cd src
%       >> run_all
%
%   Outputs (written to ../results and shown on screen):
%     * single_trial_result.mat, single_trial_timeseries.png,
%       single_trial_top_view.png   - one highlighted landing (cf. Fig. 7-8)
%     * monte_carlo_metrics.mat, monte_carlo_statistics.png
%       - 100-run touchdown statistics (cf. Fig. 9)
%
%   Hand the regenerated figures/metrics back for inclusion in the report.

thisDir = fileparts(mfilename('fullpath'));
addpath(thisDir);

params = initialize_paper_parameters();

fprintf('==============================================================\n');
fprintf(' UAV-on-boat MPC-FAA reproduction (Prochazka et al., 2024)\n');
fprintf('==============================================================\n');
if exist('quadprog', 'file') ~= 2
    warning(['quadprog not found: using the projected-gradient fallback. ' ...
             'Install the Optimization Toolbox for paper-faithful results.']);
end

% ---- 1. Single highlighted landing trial ---------------------------------
fprintf('\n[1/2] Single landing trial ...\n');
result = simulate_landing_trial(params, 1, true);
plot_landing_results(result, params, 'single_trial');
save(fullfile(params.paths.results, 'single_trial_result.mat'), 'result', 'params');

fprintf('\nTouchdown summary (single trial):\n');
fprintf('  success:              %d\n',      result.success);
fprintf('  touchdown time:       %.2f s\n',  result.touchdownTime);
fprintf('  position deviation:   %.3f m\n',  result.metrics.positionDeviation);
fprintf('  horizontal velocity:  %.3f m/s\n',result.metrics.horizontalVelocityDeviation);
fprintf('  roll deviation:       %.3f deg\n',rad2deg(result.metrics.rollDeviation));
fprintf('  pitch deviation:      %.3f deg\n',rad2deg(result.metrics.pitchDeviation));
fprintf('  yaw deviation:        %.3f deg\n',rad2deg(result.metrics.yawDeviation));

% ---- 2. 100-run Monte Carlo ----------------------------------------------
fprintf('\n[2/2] Monte Carlo (100 runs) ...\n');
nRuns = 100;
metrics = struct('positionDeviation', zeros(nRuns,1), ...
    'horizontalVelocityDeviation', zeros(nRuns,1), ...
    'rollDeviation', zeros(nRuns,1), 'pitchDeviation', zeros(nRuns,1), ...
    'yawDeviation', zeros(nRuns,1), 'touchdownTime', zeros(nRuns,1), ...
    'success', false(nRuns,1));
for i = 1:nRuns
    r = simulate_landing_trial(params, i, false);
    metrics.positionDeviation(i)            = r.metrics.positionDeviation;
    metrics.horizontalVelocityDeviation(i)  = r.metrics.horizontalVelocityDeviation;
    metrics.rollDeviation(i)                = rad2deg(r.metrics.rollDeviation);
    metrics.pitchDeviation(i)               = rad2deg(r.metrics.pitchDeviation);
    metrics.yawDeviation(i)                 = rad2deg(r.metrics.yawDeviation);
    metrics.touchdownTime(i)                = r.touchdownTime;
    metrics.success(i)                      = r.success;
end
save(fullfile(params.paths.results, 'monte_carlo_metrics.mat'), 'metrics', 'params');
plot_monte_carlo_statistics(metrics, params);

fprintf('\nMonte Carlo summary (mean +/- std over %d runs):\n', nRuns);
fprintf('  success rate:         %.0f %%\n',     100*mean(metrics.success));
fprintf('  position deviation:   %.3f +/- %.3f m\n',   mean(metrics.positionDeviation), std(metrics.positionDeviation));
fprintf('  horizontal velocity:  %.3f +/- %.3f m/s\n', mean(metrics.horizontalVelocityDeviation), std(metrics.horizontalVelocityDeviation));
fprintf('  roll deviation:       %.2f +/- %.2f deg\n', mean(metrics.rollDeviation), std(metrics.rollDeviation));
fprintf('  pitch deviation:      %.2f +/- %.2f deg\n', mean(metrics.pitchDeviation), std(metrics.pitchDeviation));
fprintf('  yaw deviation:        %.2f +/- %.2f deg\n', mean(metrics.yawDeviation), std(metrics.yawDeviation));
fprintf('\nDone. Figures and .mat files written to %s\n', params.paths.results);
end
