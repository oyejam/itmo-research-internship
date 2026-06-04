function y = simulink_landing_step(t)
%SIMULINK_LANDING_STEP One executable MPC-FAA simulation step for Simulink.
% Output:
% [time; UAV 12 states; USV 12 states; mode index; deck distance; success]

persistent params model pred x uPrev memory landed trial

if isempty(params) || t == 0
    params = initialize_paper_parameters();
    model = build_uav_linear_model(params);
    pred = build_prediction_matrices(model, params);
    trial = 1;
    x = zeros(12,1);
    x(1:3) = [-42; -18; 2.0];
    uPrev = zeros(4,1);
    memory = [];
    landed = false;
end

usvNow = gerstner_usv_motion(t, params, trial);
[mode, memory] = uav_landing_state_machine(t, x, usvNow, memory, params);
deckDistance = x(3) - usvNow.position(3);

if ~landed && ~strcmp(mode, 'LANDED')
    usvPred = generate_usv_prediction(t, params, trial);
    ref = create_reference_trajectory(x, usvPred, mode, params);
    [du, ~] = solve_mpc_qp(x, uPrev, ref, params, model, pred, mode, deckDistance);
    uPrev = min(max(uPrev + du, ...
        (params.uav.minOmegaSq - params.uav.hoverOmegaSq) * ones(4,1)), ...
        (params.uav.maxOmegaSq - params.uav.hoverOmegaSq) * ones(4,1));
    x = model.A * x + model.B * uPrev;
    x(6) = mod(x(6) + pi, 2*pi) - pi;
else
    landed = true;
end

y = [t; x; usvNow.fullState; mode_to_index_local(mode); deckDistance; double(landed)];
end

function idx = mode_to_index_local(mode)
names = {'GET_ALTITUDE','APPROACH','TRACKING','TRACKING_STABLE','DESCENT','FLARE','LANDED'};
idx = find(strcmp(names, mode), 1, 'first');
if isempty(idx)
    idx = 0;
end
end

