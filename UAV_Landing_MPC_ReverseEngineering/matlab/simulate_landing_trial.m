function result = simulate_landing_trial(params, trial, verbose)
%SIMULATE_LANDING_TRIAL Closed-loop MPC-FAA simulation.

if nargin < 3
    verbose = false;
end

model = build_uav_linear_model(params);
pred = build_prediction_matrices(model, params);

% UAV starts far from the USV, matching the random initial-distance setup.
rng(trial);
x = zeros(12,1);
x(1:3) = [-42 -18 2.0]' + [5*randn; 3*randn; 0];
x(6) = 0;
uPrev = zeros(4,1); % deviation from hover rotor squared speed
memory = [];

log.t = [];
log.x = [];
log.u = [];
log.usv = [];
log.mode = {};
log.deckDistance = [];
log.exitflag = [];

for step = 1:params.sim.maxSteps
    t = (step-1) * params.sim.dt;
    usvNow = gerstner_usv_motion(t, params, trial);
    [mode, memory] = uav_landing_state_machine(t, x, usvNow, memory, params);
    deckDistance = x(3) - usvNow.position(3);

    log.t(end+1,1) = t;
    log.x(end+1,:) = x';
    log.u(end+1,:) = (uPrev + model.hoverInput)';
    log.usv(end+1,:) = usvNow.fullState';
    log.mode{end+1,1} = mode;
    log.deckDistance(end+1,1) = deckDistance;

    if strcmp(mode, 'LANDED')
        break;
    end

    usvPred = generate_usv_prediction(t, params, trial);
    ref = create_reference_trajectory(x, usvPred, mode, params);
    [du, info] = solve_mpc_qp(x, uPrev, ref, params, model, pred, mode, deckDistance);
    uPrev = clamp(uPrev + du, ...
        (params.uav.minOmegaSq - params.uav.hoverOmegaSq) * ones(4,1), ...
        (params.uav.maxOmegaSq - params.uav.hoverOmegaSq) * ones(4,1));
    x = model.A * x + model.B * uPrev;
    x(6) = wrapToPiLocal(x(6));
    log.exitflag(end+1,1) = info.exitflag;

    if verbose && mod(step, 50) == 0
        fprintf('t=%5.1f mode=%-15s deckDistance=%5.2f horizontalError=%5.2f\n', ...
            t, mode, deckDistance, norm(x(1:2) - usvNow.position(1:2)));
    end
end

lastIdx = size(log.x, 1);
touchX = log.x(lastIdx,:)';
touchUSV = log.usv(lastIdx,:)';
metrics.positionDeviation = norm(touchX(1:2) - touchUSV(1:2));
metrics.horizontalVelocityDeviation = norm(touchX(7:8) - touchUSV(7:8));
metrics.verticalVelocity = touchX(9) - touchUSV(9);
metrics.rollDeviation = angleDiff(touchX(4), touchUSV(4));
metrics.pitchDeviation = angleDiff(touchX(5), touchUSV(5));
metrics.yawDeviation = angleDiff(touchX(6), touchUSV(6));

result.log = log;
result.metrics = metrics;
result.success = strcmp(log.mode{end}, 'LANDED') && ...
    metrics.positionDeviation < 1.0 && abs(metrics.verticalVelocity) < 1.2;
result.touchdownTime = log.t(end);
end

function y = clamp(x, xmin, xmax)
y = min(max(x, xmin), xmax);
end

function a = angleDiff(a1, a2)
a = wrapToPiLocal(a1 - a2);
end

function a = wrapToPiLocal(a)
a = mod(a + pi, 2*pi) - pi;
end

