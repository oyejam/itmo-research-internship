function [Ad, Bd] = discretize_uav_model(Ac, Bc, Ts)
%DISCRETIZE_UAV_MODEL Exact zero-order-hold discretization using expm.

nx = size(Ac, 1);
nu = size(Bc, 2);
M = expm([Ac Bc; zeros(nu, nx + nu)] * Ts);
Ad = M(1:nx, 1:nx);
Bd = M(1:nx, nx+1:nx+nu);
end

