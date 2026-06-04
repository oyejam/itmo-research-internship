function pred = build_prediction_matrices(model, params)
%BUILD_PREDICTION_MATRICES Prediction matrices for augmented delta-input MPC.

A = model.A;
B = model.B;
C = model.C;
nx = model.nx;
nu = model.nu;
ny = model.ny;
N = params.mpc.N;

Aa = [A B; zeros(nu, nx) eye(nu)];
Ba = [B; eye(nu)];
Ca = [C zeros(ny, nu)];
nxa = nx + nu;

PhiX = zeros(N*nxa, nxa);
GammaX = zeros(N*nxa, N*nu);
for i = 1:N
    rows = (i-1)*nxa + (1:nxa);
    PhiX(rows,:) = Aa^i;
    for j = 1:i
        cols = (j-1)*nu + (1:nu);
        GammaX(rows, cols) = Aa^(i-j) * Ba;
    end
end

Cbar = kron(eye(N), Ca);
PhiY = Cbar * PhiX;
GammaY = Cbar * GammaX;

pred.Aa = Aa;
pred.Ba = Ba;
pred.Ca = Ca;
pred.PhiX = PhiX;
pred.GammaX = GammaX;
pred.PhiY = PhiY;
pred.GammaY = GammaY;
pred.nxa = nxa;
end

