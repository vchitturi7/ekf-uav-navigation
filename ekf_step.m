function [x_est, P] = ekf_step(x_est, P, u, z, dt, m, g, Q, R, H)
% ekf_step.m
% Runs one full EKF predict and update cycle.
%
% Inputs:
%   x_est = current state estimate [px, pz, vx, vz, theta] (5x1)
%   P     = current covariance matrix (5x5)
%   u     = control input [T] thrust (1x1)
%   z     = GPS measurement [px_meas, pz_meas] (2x1)
%   dt    = timestep (s)
%   m     = mass (kg)
%   g     = gravity (m/s^2)
%   Q     = process noise covariance (5x5)
%   R     = measurement noise covariance (2x2)
%   H     = measurement matrix (2x5)
%
% Outputs:
%   x_est = corrected state estimate (5x1)
%   P     = corrected covariance matrix (5x5)

% PREDICT
x_pred = state_transition(x_est, u, dt, m, g);
F      = compute_jacobian(x_est, u, dt, m);
P_pred = F * P * F' + Q;

% UPDATE
y = z - H * x_pred;              % innovation
S = H * P_pred * H' + R;         % innovation covariance
K = P_pred * H' / S;             % Kalman gain
x_est = x_pred + K * y;          % corrected state
P     = (eye(5) - K*H) * P_pred; % corrected covariance

end