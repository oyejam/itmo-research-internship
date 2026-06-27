function params = initialize_paper_parameters()
%INITIALIZE_PAPER_PARAMETERS Parameters taken from, or tuned to match, the paper.

root = fileparts(fileparts(mfilename('fullpath')));
params.paths.root = root;
params.paths.results = fullfile(root, 'results');

% Paper Table 3.
params.stateMachine.approachAltitude = 15.0;  % h_a [m]
params.stateMachine.trackingAltitude = 7.0;   % h_t [m]
params.stateMachine.landingVelocity = 1.0;    % v_la [m/s]
params.stateMachine.flareVelocity = 0.5;      % v_fa [m/s]
params.stateMachine.flareHeight = 1.2;        % chosen from paper description
params.stateMachine.horizontalLandingTolerance = 0.7;
params.stateMachine.visualRange = 18.0;
params.stateMachine.touchdownHeight = 0.10;
params.stateMachine.maxTime = 70.0;

% Paper Section 5.3: 2 s horizon, 20 prediction steps.
params.mpc.N = 20;
params.mpc.Ts = 0.1;
params.sim.dt = params.mpc.Ts;
params.sim.maxSteps = ceil(params.stateMachine.maxTime / params.sim.dt);

% Soft state-constraint penalties (slack variables in solve_mpc_qp.m). Hard
% state constraints over the horizon can become infeasible during aggressive
% transients; softening them guarantees the QP always returns a usable move and
% makes the velocity/attitude limits act as strong penalties rather than walls.
params.mpc.softWeight = 1e5;      % linear penalty on constraint-violation slack
params.mpc.softWeightL2 = 1e-3;   % small quadratic regularisation on the slack

% UAV dimensions from Section 5.1. Remaining values are representative for
% a 3.5 kg multirotor and intentionally easy to tune.
params.uav.mass = 3.5;
params.uav.armLength = 0.325;
params.uav.height = 0.15;
params.uav.g = 9.80665;
params.uav.Ixx = 0.082;
params.uav.Iyy = 0.084;
params.uav.Izz = 0.145;
params.uav.kT = 1.9e-5;
params.uav.drag = 2.6e-7;
% Physical hover rotor-speed-squared (per motor). Used ONLY to scale the input
% matrix in build_uav_linear_model.m. The MPC works in NORMALISED input units
% u = omega^2 / hoverOmegaSqPhysical, so that hover = 1 per motor; this keeps
% the increment penalty R = 0.1 (Table 4) correctly scaled. Without this
% normalisation the raw omega^2 (~4.5e5) makes any thrust increment
% astronomically expensive and the UAV cannot climb or manoeuvre.
params.uav.hoverOmegaSqPhysical = params.uav.mass * params.uav.g / (4 * params.uav.kT);
params.uav.hoverOmegaSq = 1.0;       % normalised hover thrust (per motor)
params.uav.minOmegaSq = 0.15;        % normalised lower rotor-speed-squared limit
params.uav.maxOmegaSq = 2.20;        % normalised upper rotor-speed-squared limit
params.uav.maxDeltaOmegaSq = 0.35;   % normalised per-step slew limit

% Table 4 constraints.
params.constraints.rollPitchMax = 0.7854;
params.constraints.xyVelocityMax = 8.0;
params.constraints.zVelocityMax = 4.0;
params.constraints.eulerRateMax = 2.0;

% Table 4 weighting. The exponential terms are evaluated in
% faa_weight_matrix.m using the current vertical deck distance.
params.weights.Rdu = 0.1 * eye(4);
params.weights.P = zeros(12);
params.weights.baseQDiag = [30 30 40 1 1 50 1 1 3000 1 1 1];
params.weights.faa.zAlpha = 10000;
params.weights.faa.zBeta = 20;
params.weights.faa.attAlpha = 50000;
params.weights.faa.attBeta = 10;
params.weights.faa.vzAlpha = 3000;
params.weights.faa.vzBeta = 25;
params.weights.useFAA = true;

% USV and waves from Section 5.1. Moderate sea nominal case.
params.usv.length = 5.0;
params.usv.width = 2.5;
params.usv.deckLength = 2.5;
params.usv.deckWidth = 1.7;
params.usv.deckHeight = 1.3;
params.usv.currentVelocity = [0.15; -0.06; 0];
params.usv.pathVelocity = 0.0;
params.usv.initialPosition = [0; 0; 0];
params.usv.initialYaw = deg2rad(20);
params.usv.wave.amplitude = [0.50 0.18 0.12];
params.usv.wave.period = [5.0 7.0 3.6];
params.usv.wave.direction = deg2rad([0 55 115]);
params.usv.wave.phase = [0.0 1.7 4.2];
params.usv.wave.steepness = [0.65 0.35 0.25];

params.noise.positionStd = 0.03;
params.noise.attitudeStd = deg2rad(0.25);

if ~exist(params.paths.results, 'dir')
    mkdir(params.paths.results);
end
end

