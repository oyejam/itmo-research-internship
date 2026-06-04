function [mode, memory] = uav_landing_state_machine(t, x, usv, memory, params)
%UAV_LANDING_STATE_MACHINE Paper-inspired NAVIGATION/FOLLOW/LANDING logic.

if isempty(memory)
    memory.mode = 'GET_ALTITUDE';
    memory.touchdown = false;
end

rel = x(1:3) - usv.position;
horizontalError = norm(rel(1:2));
deckDistance = x(3) - usv.position(3);

switch memory.mode
    case 'GET_ALTITUDE'
        if abs(x(3) - params.stateMachine.approachAltitude) < 0.6
            memory.mode = 'APPROACH';
        end
    case 'APPROACH'
        if horizontalError < params.stateMachine.visualRange
            memory.mode = 'TRACKING';
        end
    case 'TRACKING'
        if horizontalError < 1.2 && abs(deckDistance - params.stateMachine.trackingAltitude) < 0.8
            memory.mode = 'TRACKING_STABLE';
            memory.stableSince = t;
        end
    case 'TRACKING_STABLE'
        stableTime = t - memory.stableSince;
        if horizontalError < params.stateMachine.horizontalLandingTolerance && stableTime > 1.0
            memory.mode = 'DESCENT';
        end
    case 'DESCENT'
        if horizontalError > 2.0
            memory.mode = 'TRACKING';
        elseif deckDistance < params.stateMachine.flareHeight
            memory.mode = 'FLARE';
        end
    case 'FLARE'
        if deckDistance < params.stateMachine.touchdownHeight && abs(x(9) - usv.velocity(3)) < 1.2
            memory.mode = 'LANDED';
            memory.touchdown = true;
        end
    case 'LANDED'
        memory.touchdown = true;
end

mode = memory.mode;
end

