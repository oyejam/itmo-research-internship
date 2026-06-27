function Q = faa_weight_matrix(params, verticalDistance, flightState)
%FAA_WEIGHT_MATRIX  Time-varying state-weighting matrix Q(t) with the paper's
%   Forcing Attitude Alignment (FAA) terms (Prochazka et al. 2024, Table 4,
%   Eqs. 29 and 33).
%
%   The base diagonal reproduces Table 4 exactly:
%     diag([30 30 40 1 1 50 1 1 3000 1 1 1])
%   and, during the LANDING phase, the FAA exponential terms are added to the
%   z-position, roll, pitch and z-velocity weights as the vertical deck
%   distance d shrinks:
%     q(z)     = 40   + 10000 / e^{20 d}
%     q(roll)  = 1    + 50000 / e^{10 d}
%     q(pitch) = 1    + 50000 / e^{10 d}
%     q(vz)    = 3000 + 3000  / e^{25 d}
%
%   flightState is one of the granular state-machine states:
%     GET_ALTITUDE, APPROACH, TRACKING, TRACKING_STABLE, DESCENT, FLARE, LANDED
%   which are mapped here to the paper's three mission phases
%   (NAVIGATION / FOLLOW / LANDING) to select the phase-dependent weighting.

d = max(0, verticalDistance);
q = params.weights.baseQDiag;

phase = state_to_phase(flightState);

% --- LANDING phase: Forcing Attitude Alignment ----------------------------
if params.weights.useFAA && strcmp(phase, 'LANDING')
    q(3) = 40   + params.weights.faa.zAlpha   / exp(params.weights.faa.zBeta   * d);
    q(4) = 1    + params.weights.faa.attAlpha / exp(params.weights.faa.attBeta * d);
    q(5) = 1    + params.weights.faa.attAlpha / exp(params.weights.faa.attBeta * d);
    q(9) = 3000 + params.weights.faa.vzAlpha  / exp(params.weights.faa.vzBeta  * d);
end

% --- Phase-dependent reference prioritisation -----------------------------
switch phase
    case 'NAVIGATION'
        % Prioritise reaching the approach point; relax attitude tracking.
        q([4 5 10 11 12]) = [0.5 0.5 0.1 0.1 0.2];
        q([7 8 9])        = [0.1 0.1 0.5];
    case 'FOLLOW'
        % Increase penalisation of the UAV/USV linear-velocity difference so
        % the UAV follows the moving deck (paper Sec. 4.1, 4.3).
        q([7 8]) = [60 60];
        q(9)     = 800;
end

% In FLARE specifically, also strongly track yaw and Euler rates so the UAV
% lands aligned on all legs (paper Sec. 4.3, flare maneuver).
if strcmp(flightState, 'FLARE')
    q(6)     = 100;
    q(10:12) = [30 30 30];
end

Q = diag(q);
end

function phase = state_to_phase(flightState)
%STATE_TO_PHASE Map a granular state-machine state to a mission phase.
switch flightState
    case {'GET_ALTITUDE', 'APPROACH'}
        phase = 'NAVIGATION';
    case {'TRACKING', 'TRACKING_STABLE'}
        phase = 'FOLLOW';
    case {'DESCENT', 'FLARE'}
        phase = 'LANDING';
    otherwise
        phase = 'NAVIGATION';
end
end
