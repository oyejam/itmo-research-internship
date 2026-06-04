function Q = faa_weight_matrix(params, verticalDistance, flightState)
%FAA_WEIGHT_MATRIX Dynamic Q(t) matrix with paper-style FAA terms.

d = max(0, verticalDistance);
q = params.weights.baseQDiag;

isLanding = strcmp(flightState, 'DESCENT') || strcmp(flightState, 'FLARE');
if params.weights.useFAA && isLanding
    q(3) = 40 + params.weights.faa.zAlpha / exp(params.weights.faa.zBeta * d);
    q(4) = 1 + params.weights.faa.attAlpha / exp(params.weights.faa.attBeta * d);
    q(5) = 1 + params.weights.faa.attAlpha / exp(params.weights.faa.attBeta * d);
    q(9) = 3000 + params.weights.faa.vzAlpha / exp(params.weights.faa.vzBeta * d);
end

if strcmp(flightState, 'NAVIGATION')
    q([4 5 10 11 12]) = [0.5 0.5 0.1 0.1 0.2];
    q([7 8 9]) = [0.1 0.1 0.5];
elseif strcmp(flightState, 'FOLLOW') || strcmp(flightState, 'TRACKING_STABLE')
    q([7 8]) = [60 60];
    q(9) = 800;
elseif strcmp(flightState, 'FLARE')
    q(6) = 100;
    q(10:12) = [30 30 30];
end

Q = diag(q);
end

