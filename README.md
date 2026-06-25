# 3D EKF-Based UAV Navigation Filter

## Motivation

I built this project to develop a concrete understanding of state estimation and sensor fusion. The goal was to implement a Kalman filter from scratch, deriving the math by hand before writing any code.

## Overview

A 3D Extended Kalman Filter (EKF) for UAV position, velocity, and attitude estimation, implemented in MATLAB, Simulink, and C++. The filter fuses low-rate GPS position measurements with high-rate drone dynamics to estimate states that GPS cannot directly measure, including velocity and attitude.

A GPS dropout scenario is included to demonstrate dead-reckoning behavior during signal loss and automatic re-convergence on GPS recovery.

## Technical Details

**State vector (8 states):**

```
[px, py, pz, vx, vy, vz, theta, phi]
```

Position, velocity, pitch angle, roll angle

**Measurement model:**

GPS measures position only (px, py, pz). Velocity and attitude are inferred by the filter from position history.

**EKF predict/update cycle:**

```
Predict:
  x_pred = f(x, u)          % nonlinear state transition
  F      = df/dx             % Jacobian, recomputed each timestep
  P_pred = F*P*F' + Q        % covariance propagation

Update:
  y     = z - H*x_pred       % innovation
  S     = H*P*H' + R         % innovation covariance
  K     = P*H'/S             % Kalman gain
  x_est = x_pred + K*y       % corrected state
  P     = (I - K*H)*P        % updated covariance
```

**Jacobian derivation:**

The state transition is nonlinear because thrust-to-acceleration coupling depends on attitude angles (theta, phi) via sin/cos terms. The 8x8 Jacobian F is derived analytically by differentiating each state equation with respect to each state variable, then recomputed at every timestep around the current state estimate.

**GPS dropout:**

Between t=3s and t=5s the GPS signal is cut. During dropout the filter runs predict-only, dead-reckoning via dynamics propagation with no measurement corrections. Uncertainty grows during dropout and the filter re-converges when GPS returns.

## File Structure

**MATLAB implementation:**
- `compute_jacobian.m` - Analytical Jacobian of drone dynamics
- `ekf_step.m` - Full EKF predict/update cycle
- `state_transition.m` - Nonlinear drone state transition
- `params.m` - All simulation constants
- `run_simulation.m` - 2D MATLAB simulation with GPS dropout plots
- `plot_results.m` - Plotting script for Simulink output
- `ekf_drone_3d.slx` - Simulink model: drone plant, GPS sensor, EKF block

**C++ implementation:**
- `ekf.h` - EKF class definition
- `ekf.cpp` - Predict/update implementation using Eigen
- `main.cpp` - Simulation loop and CSV output

## MATLAB vs Simulink

The MATLAB scripts implement a 2D version (5-state) used to validate the filter math before building in Simulink. The Simulink model implements the full 3D version (8-state) as a block diagram with a drone dynamics subsystem, GPS sensor with noise, dropout logic, and EKF feedback loops.

## C++ Port

The filter was ported to C++ using the Eigen library, implementing the full predict-update cycle, Jacobian computation, and GPS dropout logic in a compiled environment. Results are written to `results.csv` for comparison against the MATLAB baseline.

Build and run:

```bash
g++ -I path/to/eigen main.cpp ekf.cpp -o ekf_sim
./ekf_sim
```

## How to Run

**MATLAB (2D validation):**
```matlab
run_simulation
```

**Simulink (3D full model):**
1. Open `ekf_drone_3d.slx` in MATLAB
2. Press Run (Ctrl+T)
3. Run `plot_results.m` to generate plots

## Dependencies

- MATLAB R2024a or later with Simulink
- C++: Eigen 3.x (header-only, no installation required)
