# Paper Alignment Notes

## Implemented paper elements

- UAV state vector follows Eq. (4)-(5): position, attitude, linear velocity, and Euler rates.
- UAV hover-linearized model follows Eq. (12)-(20). The implementation uses the common small-angle hover approximation for horizontal acceleration and the rotor-squared input mapping for vertical/attitude acceleration.
- MPC formulation follows Eq. (21)-(32), including the augmented previous-input state and delta-input decision variable.
- FAA follows Eq. (33), using exponential growth of roll/pitch and vertical weights as deck distance decreases.
- Mission phases follow Fig. 3: NAVIGATION, FOLLOW, and LANDING with Get altitude, Approach, Tracking, Tracking stable, Descent, Flare, and Landed states.
- Wave-driven USV motion follows the Gerstner wave idea of Eq. (34)-(35), using three components as stated in the paper.
- Main simulation parameters follow Table 3 and Table 4.

## Missing details in the paper and chosen approximations

The paper relies on the CTU MRS UAV system, Gazebo, Pixhawk attitude-rate control, AprilTag/UVDAR sensing, and a separate USV state-estimation paper. Those full implementations and identified hydrodynamic matrices are not included in the article. For a self-contained MATLAB reverse-engineering project:

- The USV is represented by a kinematic Gerstner-wave deck trajectory rather than a full Fossen hydrodynamic model.
- Sensor fusion is represented by direct predicted USV state access; optional noise parameters are provided for extension.
- The low-level Pixhawk/MRS reference tracker is represented by applying the MPC-generated rotor-squared input to the linearized UAV model.

These approximations keep the project faithful to the paper's reproducible mathematical core while remaining executable in MATLAB R2021a.

