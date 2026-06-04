# Reverse Engineering Solution

Paper reproduced:

`Ondrej Prochazka et al., "Model predictive control-based trajectory generation for agile landing of unmanned aerial vehicle on a moving boat", Ocean Engineering, 2024.`

This folder contains a MATLAB R2021a-compatible reverse-engineering implementation of the paper's MPC-based UAV landing trajectory generator.

## What is implemented

- 12-state UAV model linearized about hover: position, Euler attitude, linear velocity, and Euler rates.
- Input-increment MPC, following the paper's augmented model formulation.
- 2 s prediction horizon with 20 prediction steps.
- Time-varying MPC weights with Forcing Attitude Alignment (FAA).
- NAVIGATION, FOLLOW, and LANDING state-machine phases.
- Gerstner-wave-inspired USV heave/roll/pitch/surge/sway/yaw prediction.
- Monte Carlo landing tests and paper-style touchdown statistics.
- A MATLAB script that builds an executable Simulink harness for MATLAB R2021a.

## How to run

Open MATLAB R2021a in this folder and run:

```matlab
cd('matlab')
run_main
```

Optional:

```matlab
run_monte_carlo
build_simulink_model
```

`run_main` produces plots and saves results in `../results`.

For a more detailed explanation of how each paper section maps to the code, read `docs/reverse_engineering_report.md` and `docs/paper_alignment_notes.md`.

## Notes on fidelity

The paper does not publish all proprietary MRS/Gazebo implementation details, sensor-fusion code, or exact USV hydrodynamic matrices. This solution therefore implements the parts that are fully specified in the paper and uses explicitly documented engineering approximations for missing parameters:

- The UAV is represented by the paper's hover-linearized multirotor model.
- The USV motion is generated from three wave components as described in the simulation section, with a compact kinematic approximation for deck roll and pitch.
- The MPC structure, horizon, state constraints, dynamic weighting, and FAA behavior follow the paper.

This makes the project suitable for reverse-engineering study, parameter tuning, and MATLAB/Simulink demonstration.
