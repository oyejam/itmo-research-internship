function usv = gerstner_usv_motion(t, params, trial)
%GERSTNER_USV_MOTION Compact USV deck trajectory inspired by Eq. (34)-(35).

if nargin < 3
    trial = 1;
end
rng(trial);

A = params.usv.wave.amplitude(:);
T = params.usv.wave.period(:);
omega = 2*pi ./ T;
dir = params.usv.wave.direction(:);
phase = params.usv.wave.phase(:) + 0.1 * trial;
q = params.usv.wave.steepness(:);

base = params.usv.initialPosition + params.usv.currentVelocity * t;
if params.usv.pathVelocity > 0
    base = base + square_path_position(t, params.usv.pathVelocity);
end

z = 0; zd = 0; xWave = 0; yWave = 0; xdWave = 0; ydWave = 0;
roll = 0; pitch = 0; rolld = 0; pitchd = 0;
for i = 1:numel(A)
    arg = -omega(i) * t + phase(i);
    xWave = xWave - q(i) * cos(dir(i)) * A(i) * sin(arg);
    yWave = yWave - q(i) * sin(dir(i)) * A(i) * sin(arg);
    xdWave = xdWave + q(i) * cos(dir(i)) * A(i) * omega(i) * cos(arg);
    ydWave = ydWave + q(i) * sin(dir(i)) * A(i) * omega(i) * cos(arg);
    z = z + A(i) * cos(arg);
    zd = zd + A(i) * omega(i) * sin(arg);
    roll = roll + 0.18 * A(i) * sin(dir(i)) * sin(arg);
    pitch = pitch + 0.18 * A(i) * cos(dir(i)) * sin(arg);
    rolld = rolld - 0.18 * A(i) * sin(dir(i)) * omega(i) * cos(arg);
    pitchd = pitchd - 0.18 * A(i) * cos(dir(i)) * omega(i) * cos(arg);
end

yaw = params.usv.initialYaw + 0.04 * sin(0.22*t);
yawd = 0.04 * 0.22 * cos(0.22*t);

usv.position = [base(1) + xWave; base(2) + yWave; params.usv.deckHeight + z];
usv.attitude = [roll; pitch; yaw];
usv.velocity = [params.usv.currentVelocity(1) + xdWave; params.usv.currentVelocity(2) + ydWave; zd];
usv.eulerRate = [rolld; pitchd; yawd];
usv.fullState = [usv.position; usv.attitude; usv.velocity; usv.eulerRate];
end

function p = square_path_position(t, speed)
side = 25;
period = 4 * side / speed;
s = mod(t, period) * speed;
if s < side
    p = [s; 0; 0];
elseif s < 2*side
    p = [side; s-side; 0];
elseif s < 3*side
    p = [side-(s-2*side); side; 0];
else
    p = [0; side-(s-3*side); 0];
end
end

