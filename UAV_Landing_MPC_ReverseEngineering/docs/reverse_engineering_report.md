# Reverse Engineering Report

## 1. Paper Objective

The paper proposes an MPC-based trajectory generator for landing a multirotor UAV on a moving and tilting USV deck. The central idea is to exploit predicted USV states rather than waiting for a nearly level deck. During the final flare maneuver, the UAV aligns roll, pitch, yaw, velocity, and position with the USV deck so touchdown occurs with low position error, low impact speed, and small attitude mismatch.

## 2. Vehicle Models

The paper uses:

- A 6-DOF Fossen-style USV model for state estimation and prediction.
- A nonlinear rigid-body UAV model.
- A hover-linearized UAV model for real-time linear MPC.

The MATLAB implementation reproduces the linear MPC model because that is the portion directly used for trajectory generation. The state vector is:

```text
x = [px py pz roll pitch yaw vx vy vz roll_rate pitch_rate yaw_rate]'
```

The control vector is the deviation of four squared rotor speeds from hover:

```text
u = [omega_1^2 omega_2^2 omega_3^2 omega_4^2]' - u_hover
```

The previous input is added to the augmented state so the QP decision variable is the input increment `du`, matching the paper's Eq. (26)-(32).

## 3. MPC Formulation

The implemented cost is:

```text
sum_k (y_k - r_k)' Q(t) (y_k - r_k) + du_k' R du_k
```

with:

- `N = 20`
- `Ts = 0.1 s`
- horizon length `2 s`
- input slew constraints
- roll/pitch, velocity, Euler-rate, altitude, and rotor-speed constraints

The implementation uses `quadprog` when available. A simple projected-gradient fallback is included so the files remain demonstrable on MATLAB installations without Optimization Toolbox, although `quadprog` is recommended for proper results.

## 4. Forcing Attitude Alignment

The paper's FAA function:

```text
f(vd) = 1 + alpha / exp(beta * vd)
```

is implemented in `faa_weight_matrix.m`. As the UAV approaches the deck, vertical-position, vertical-velocity, roll, and pitch penalties increase sharply. This forces the UAV to trade a small amount of lateral tracking aggressiveness for attitude synchronization near touchdown.

## 5. Mission State Machine

The implementation follows the paper's states:

- `GET_ALTITUDE`
- `APPROACH`
- `TRACKING`
- `TRACKING_STABLE`
- `DESCENT`
- `FLARE`
- `LANDED`

Each state changes the reference sent to the MPC. For example, `APPROACH` tracks the USV x-y location at approach altitude, `TRACKING` tracks the USV at the tracking altitude, and `FLARE` tracks the full deck pose and rates.

## 6. USV and Sea-State Model

The paper's full Gazebo and hydrodynamic setup is not fully published. This solution therefore uses a compact, three-component Gerstner-wave-inspired USV deck trajectory. The nominal setting matches the paper's Moderate sea case:

- peak amplitude around `0.5 m`
- main wave period `5 s`
- three wave components
- roll and pitch induced by wave direction and heave

This provides moving-deck position, attitude, velocity, and Euler-rate predictions for the MPC horizon.

## 7. Experimental Reproduction Targets

The paper reports, for Moderate sea and current-carried USV:

- 100 landing tests
- touchdown position deviation near `0.15 m` mean
- horizontal velocity deviation near `0.36 m/s` mean
- roll and pitch deviations mostly within a few degrees
- landing maneuver around `6 s` from descent start in the highlighted trial

Use `run_monte_carlo.m` to generate corresponding local statistics. Because the exact Gazebo, estimator, and MRS controller stack are not included in the paper, the numerical values should be tuned by adjusting parameters in `initialize_paper_parameters.m`.

