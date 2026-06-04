function model = build_uav_linear_model(params)
%BUILD_UAV_LINEAR_MODEL Hover-linearized UAV model used by the MPC.
% State order:
% [x y z phi theta psi vx vy vz vphi vtheta vpsi]'.

g = params.uav.g;
m = params.uav.mass;
l = params.uav.armLength;
kT = params.uav.kT;
b = params.uav.drag;
Ixx = params.uav.Ixx;
Iyy = params.uav.Iyy;
Izz = params.uav.Izz;

Ac = zeros(12, 12);
Ac(1, 7) = 1; Ac(2, 8) = 1; Ac(3, 9) = 1;
Ac(4,10) = 1; Ac(5,11) = 1; Ac(6,12) = 1;

% Small-angle hover linearization: roll controls y acceleration, pitch
% controls x acceleration. ENU z is up.
Ac(7,5) = g;
Ac(8,4) = -g;

Bc = zeros(12, 4);
Bc(9,:)  = kT / m;
Bc(10,:) = [-kT*l/Ixx, -kT*l/Ixx,  kT*l/Ixx,  kT*l/Ixx];
Bc(11,:) = [-kT*l/Iyy,  kT*l/Iyy,  kT*l/Iyy, -kT*l/Iyy];
Bc(12,:) = [-b/Izz,      b/Izz,     -b/Izz,      b/Izz];

[A, B] = discretize_uav_model(Ac, Bc, params.mpc.Ts);

model.Ac = Ac;
model.Bc = Bc;
model.A = A;
model.B = B;
model.C = eye(12);
model.nx = 12;
model.nu = 4;
model.ny = 12;
model.hoverInput = params.uav.hoverOmegaSq * ones(4,1);
end

