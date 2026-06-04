function pred = generate_usv_prediction(t, params, trial)
%GENERATE_USV_PREDICTION USV full-state prediction over the MPC horizon.

N = params.mpc.N;
Ts = params.mpc.Ts;
pred = zeros(N, 12);
for k = 1:N
    usv = gerstner_usv_motion(t + k*Ts, params, trial);
    pred(k,:) = usv.fullState';
end
end

