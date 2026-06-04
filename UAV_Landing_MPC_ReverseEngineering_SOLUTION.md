# MATLAB/Simulink Reverse Engineering Solution

This is the parent-folder solution file for the selected paper:

`Model predictive control-based trajectory generation for agile landing of unmanned aerial vehicle on a moving boat`

The complete MATLAB/Simulink implementation is in:

`UAV_Landing_MPC_ReverseEngineering/`

Start here:

1. Open MATLAB R2021a.
2. Set the current folder to `UAV_Landing_MPC_ReverseEngineering/matlab`.
3. Run `run_main`.
4. For repeated statistical testing, run `run_monte_carlo`.
5. To generate the Simulink executable harness, run `build_simulink_model`.

The implementation follows the paper's reproducible core:

- UAV hover-linearized 12-state model.
- Input-increment MPC using the augmented previous-input state.
- 2 s horizon with 20 prediction steps.
- Dynamic state weighting and Forcing Attitude Alignment.
- NAVIGATION, FOLLOW, and LANDING state-machine logic.
- Gerstner-wave-inspired USV deck prediction.
- Paper-style touchdown plots and Monte Carlo metrics.

Important files:

- `README.md`: execution guide and fidelity notes.
- `docs/paper_alignment_notes.md`: mapping from paper sections/equations to implementation.
- `matlab/run_main.m`: single landing experiment.
- `matlab/run_monte_carlo.m`: 100-run statistical reproduction.
- `matlab/build_simulink_model.m`: creates a Simulink executable harness in MATLAB.
- `matlab/solve_mpc_qp.m`: MPC quadratic program.
- `matlab/faa_weight_matrix.m`: dynamic FAA weighting from the paper.
