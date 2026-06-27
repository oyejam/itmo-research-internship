function [du0, info] = solve_mpc_qp(x, uPrev, refHorizon, params, model, pred, flightState, deckDistance)
%SOLVE_MPC_QP Dense QP for input-increment MPC.

N = params.mpc.N;
nx = model.nx;
nu = model.nu;
nxa = pred.nxa;

xa0 = [x; uPrev];
Q = faa_weight_matrix(params, deckDistance, flightState);
Qbar = kron(eye(N), Q);
Rbar = kron(eye(N), params.weights.Rdu);

r = reshape(refHorizon', N*nx, 1);
H = pred.GammaY' * Qbar * pred.GammaY + Rbar;
H = 0.5 * (H + H') + 1e-8 * eye(size(H));
f = pred.GammaY' * Qbar * (pred.PhiY * xa0 - r);

% Slew constraints on the input increments.
duMax = params.uav.maxDeltaOmegaSq * ones(N*nu, 1);

% State and input constraints across the horizon.
[xmin, xmax] = state_bounds(params);
uDevMin = (params.uav.minOmegaSq - params.uav.hoverOmegaSq) * ones(nu, 1);
uDevMax = (params.uav.maxOmegaSq - params.uav.hoverOmegaSq) * ones(nu, 1);

Sx = zeros(N*nx, N*nxa);
Su = zeros(N*nu, N*nxa);
for k = 1:N
    xRows = (k-1)*nx + (1:nx);
    uRows = (k-1)*nu + (1:nu);
    xaRows = (k-1)*nxa + (1:nxa);
    Sx(xRows, xaRows(1:nx)) = eye(nx);
    Su(uRows, xaRows(nx+1:nx+nu)) = eye(nu);
end

Mx = Sx * pred.GammaX;
bx = Sx * pred.PhiX * xa0;
Mu = Su * pred.GammaX;
bu = Su * pred.PhiX * xa0;

% --- Soft state constraints --------------------------------------------------
% Decision vector z = [dU; s], with one non-negative slack s per predicted
% state. The state bounds are relaxed by s and s is heavily penalised, so the
% QP is always feasible (a hard formulation goes infeasible during aggressive
% transients, after which a zero-move fallback lets the state run away).
ns  = N*nx;
ndu = N*nu;
Hs  = blkdiag(H, params.mpc.softWeightL2 * eye(ns));
fs  = [f; params.mpc.softWeight * ones(ns, 1)];

Zdu_ns = zeros(ndu, ns);
Aineq = [ ...
    eye(ndu),  Zdu_ns;          % dU <=  duMax
   -eye(ndu),  Zdu_ns;          % -dU <= duMax
    Mu,        Zdu_ns;          % input deviation upper
   -Mu,        Zdu_ns;          % input deviation lower
    Mx,       -eye(ns);         % state upper, softened by slack
   -Mx,       -eye(ns);         % state lower, softened by slack
    zeros(ns, ndu), -eye(ns)];  % s >= 0
bineq = [ ...
    duMax;
    duMax;
    repmat(uDevMax, N, 1) - bu;
   -repmat(uDevMin, N, 1) + bu;
    repmat(xmax, N, 1) - bx;
   -repmat(xmin, N, 1) + bx;
    zeros(ns, 1)];

opts = optimoptions_if_available();
if exist('quadprog', 'file') == 2
    [Z, ~, exitflag] = quadprog(Hs, fs, Aineq, bineq, [], [], [], [], [], opts);
else
    [Z, exitflag] = projected_gradient_qp(Hs, fs, Aineq, bineq, 200);
end

if isempty(Z) || exitflag <= 0
    Z = zeros(ndu + ns, 1);
end

dU  = Z(1:ndu);
du0 = dU(1:nu);
info.exitflag = exitflag;
info.Q = Q;
info.HCondition = cond(H);
info.maxSlack = max([0; Z(ndu+1:end)]);
end

function opts = optimoptions_if_available()
if exist('optimoptions', 'file') == 2
    opts = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
else
    opts = [];
end
end

function [xmin, xmax] = state_bounds(params)
xmin = -inf(12,1);
xmax = inf(12,1);
xmin(3) = -0.2;
xmax(3) = 25.0;
xmin(4:5) = -params.constraints.rollPitchMax;
xmax(4:5) = params.constraints.rollPitchMax;
xmin(7:8) = -params.constraints.xyVelocityMax;
xmax(7:8) = params.constraints.xyVelocityMax;
xmin(9) = -params.constraints.zVelocityMax;
xmax(9) = params.constraints.zVelocityMax;
xmin(10:12) = -params.constraints.eulerRateMax;
xmax(10:12) = params.constraints.eulerRateMax;
end

function [z, exitflag] = projected_gradient_qp(H, f, A, b, maxIter)
% Basic fallback when Optimization Toolbox is unavailable.
z = zeros(size(f));
L = max(eig(H));
alpha = 1 / max(L, 1);
for it = 1:maxIter
    z = z - alpha * (H*z + f);
    violation = A*z - b;
    active = find(violation > 0);
    for idx = active'
        ai = A(idx,:)';
        z = z - (A(idx,:)*z - b(idx)) / (ai'*ai + eps) * ai;
    end
end
exitflag = 1;
end

