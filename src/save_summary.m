function save_summary()
%SAVE_SUMMARY  Write a report-ready statistics summary from saved results.
%
%   Run this AFTER run_all (or run_monte_carlo) has finished. It does NOT
%   re-run the simulation: it simply loads the .mat files already written to
%   ../results and writes two human-readable summary files:
%
%       ../results/monte_carlo_summary.txt   (formatted table)
%       ../results/monte_carlo_summary.csv   (one row per metric)
%
%   Usage (from MATLAB, inside src/):
%       >> save_summary
%
%   Safe to add/run while a simulation is in progress: it only reads the
%   results folder when you call it.

thisDir    = fileparts(mfilename('fullpath'));
resultsDir = fullfile(fileparts(thisDir), 'results');
mcFile     = fullfile(resultsDir, 'monte_carlo_metrics.mat');

if exist(mcFile, 'file') ~= 2
    error('save_summary:noData', ...
        ['No monte_carlo_metrics.mat found in %s.\n' ...
         'Run run_all (or run_monte_carlo) to completion first.'], resultsDir);
end

S       = load(mcFile, 'metrics');
metrics = S.metrics;
nRuns   = numel(metrics.success);

% Metric label, data vector, unit  ----------------------------------------
rows = {
    'success_rate_pct',        100*mean(metrics.success),            '%'
    'position_deviation_m',    metrics.positionDeviation,            'm'
    'horizontal_velocity_mps', metrics.horizontalVelocityDeviation,  'm/s'
    'roll_deviation_deg',      metrics.rollDeviation,                'deg'
    'pitch_deviation_deg',     metrics.pitchDeviation,               'deg'
    'yaw_deviation_deg',       metrics.yawDeviation,                 'deg'
    'touchdown_time_s',        metrics.touchdownTime,                's'
};

% ---- Write the formatted .txt --------------------------------------------
txtFile = fullfile(resultsDir, 'monte_carlo_summary.txt');
fid = fopen(txtFile, 'w');
assert(fid > 0, 'Could not open %s for writing.', txtFile);
fprintf(fid, 'UAV-on-boat MPC-FAA reproduction - Monte Carlo summary\n');
fprintf(fid, 'Generated: %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf(fid, 'Runs: %d\n', nRuns);
fprintf(fid, '%-26s %12s %12s %8s\n', 'metric', 'mean', 'std', 'unit');
fprintf(fid, '%s\n', repmat('-', 1, 60));
for k = 1:size(rows, 1)
    data = rows{k, 2};
    if isscalar(data)         % success rate already reduced to a percentage
        fprintf(fid, '%-26s %12.3f %12s %8s\n', rows{k,1}, data, '-', rows{k,3});
    else
        fprintf(fid, '%-26s %12.3f %12.3f %8s\n', ...
            rows{k,1}, mean(data), std(data), rows{k,3});
    end
end
fclose(fid);

% ---- Write the .csv ------------------------------------------------------
csvFile = fullfile(resultsDir, 'monte_carlo_summary.csv');
fid = fopen(csvFile, 'w');
assert(fid > 0, 'Could not open %s for writing.', csvFile);
fprintf(fid, 'metric,mean,std,unit,n_runs\n');
for k = 1:size(rows, 1)
    data = rows{k, 2};
    if isscalar(data)
        fprintf(fid, '%s,%.6f,,%s,%d\n', rows{k,1}, data, rows{k,3}, nRuns);
    else
        fprintf(fid, '%s,%.6f,%.6f,%s,%d\n', ...
            rows{k,1}, mean(data), std(data), rows{k,3}, nRuns);
    end
end
fclose(fid);

fprintf('Summary written to:\n  %s\n  %s\n', txtFile, csvFile);
end
