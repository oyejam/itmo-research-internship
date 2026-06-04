function ref = create_reference_trajectory(x, usvPred, mode, params)
%CREATE_REFERENCE_TRAJECTORY Full-state MPC reference for each flight phase.

N = params.mpc.N;
ref = zeros(N, 12);

for k = 1:N
    usv = usvPred(k,:)';
    r = zeros(12,1);
    switch mode
        case 'GET_ALTITUDE'
            r(1:2) = x(1:2);
            r(3) = params.stateMachine.approachAltitude;
            r(6) = heading_to_target(x(1:2), usv(1:2));
        case 'APPROACH'
            r(1:2) = usv(1:2);
            r(3) = params.stateMachine.approachAltitude;
            r(6) = heading_to_target(x(1:2), usv(1:2));
        case {'TRACKING', 'TRACKING_STABLE'}
            r(1:2) = usv(1:2);
            r(3) = usv(3) + params.stateMachine.trackingAltitude;
            r(6) = usv(6);
            r(7:9) = usv(7:9);
            r(9) = 0;
        case 'DESCENT'
            desiredDistance = max(params.stateMachine.flareHeight, ...
                x(3) - usv(3) - params.stateMachine.landingVelocity * k * params.mpc.Ts);
            r(1:2) = usv(1:2);
            r(3) = usv(3) + desiredDistance;
            r(6) = usv(6);
            r(7:8) = usv(7:8);
            r(9) = usv(9) - params.stateMachine.landingVelocity;
        case 'FLARE'
            desiredDistance = max(0.02, ...
                x(3) - usv(3) - params.stateMachine.flareVelocity * k * params.mpc.Ts);
            r(1:3) = [usv(1:2); usv(3) + desiredDistance];
            r(4:6) = usv(4:6);
            r(7:9) = usv(7:9);
            r(9) = usv(9) - params.stateMachine.flareVelocity;
            r(10:12) = usv(10:12);
        otherwise
            r = x;
    end
    ref(k,:) = r';
end
end

function yaw = heading_to_target(p, target)
delta = target(:) - p(:);
yaw = atan2(delta(2), delta(1));
end

