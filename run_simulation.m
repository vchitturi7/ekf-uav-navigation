% run_simulation.m
% Main script. Run this file.
% Simulates drone trajectory, runs EKF, and plots results.
% Includes GPS dropout demonstration.

clear; close all; clc;

% Load parameters
params;

% GPS dropout window (in timesteps)
dropout_start = 300;   % timestep when GPS cuts out (t = 3s)
dropout_end   = 500;   % timestep when GPS comes back (t = 5s)

% Initialize true state and estimate
x_true = x0;
x_est  = x0 + randn(5,1)*0.1;
P      = P0;

% Control input — hover thrust
u = [T_hover];

% Preallocate history arrays
true_hist  = zeros(5, N);
est_hist   = zeros(5, N);
meas_hist  = zeros(2, N);
gps_active = zeros(1, N);

% Simulation loop
for k = 1:N

    % Propagate true state with small process noise
    x_true = state_transition(x_true, u, dt, m, g) ...
             + randn(5,1) .* [0.01; 0.01; 0.05; 0.05; 0.001];

    % Noisy GPS measurement
    z = H * x_true + randn(2,1) * 2.0;

    % GPS dropout — cut signal between dropout_start and dropout_end
    if k >= dropout_start && k <= dropout_end
        % No GPS — predict only, skip update
        x_pred = state_transition(x_est, u, dt, m, g);
        F      = compute_jacobian(x_est, u, dt, m);
        P      = F * P * F' + Q;
        x_est  = x_pred;
        gps_active(k) = 0;
    else
        % Normal EKF with GPS
        [x_est, P] = ekf_step(x_est, P, u, z, dt, m, g, Q, R, H);
        gps_active(k) = 1;
    end

    % Store history
    true_hist(:,k)  = x_true;
    est_hist(:,k)   = x_est;
    meas_hist(:,k)  = z;

end

% Time vector
t = (1:N) * dt;

% Dropout time bounds
dropout_t_start = dropout_start * dt;
dropout_t_end   = dropout_end * dt;

% -------------------------------------------------------
% Plot 1 — position trajectory
% -------------------------------------------------------
figure;
plot(true_hist(1,:), true_hist(2,:), 'b', 'LineWidth', 2, ...
     'DisplayName', 'True');
hold on;
plot(est_hist(1,:), est_hist(2,:), 'r', 'LineWidth', 1.5, ...
     'DisplayName', 'EKF estimate');
plot(meas_hist(1,:), meas_hist(2,:), 'g.', 'MarkerSize', 4, ...
     'DisplayName', 'GPS measurement');
legend; grid on;
xlabel('x position (m)');
ylabel('z position (m)');
title('EKF 2D Drone Position Estimation');

% -------------------------------------------------------
% Plot 2 — position error over time
% -------------------------------------------------------
figure;
subplot(2,1,1);
plot(t, true_hist(1,:) - est_hist(1,:), 'r', ...
     'DisplayName', 'EKF x error');
hold on;
plot(t, true_hist(1,:) - meas_hist(1,:), 'g', ...
     'DisplayName', 'GPS x error');
legend; grid on;
ylabel('x error (m)');
title('Position Estimation Error Over Time');

subplot(2,1,2);
plot(t, true_hist(2,:) - est_hist(2,:), 'r', ...
     'DisplayName', 'EKF z error');
hold on;
plot(t, true_hist(2,:) - meas_hist(2,:), 'g', ...
     'DisplayName', 'GPS z error');
legend; grid on;
ylabel('z error (m)');
xlabel('time (s)');

% -------------------------------------------------------
% Plot 3 — velocity estimates
% -------------------------------------------------------
figure;
subplot(2,1,1);
plot(t, true_hist(3,:), 'b', 'DisplayName', 'True vx');
hold on;
plot(t, est_hist(3,:), 'r', 'DisplayName', 'EKF vx');
legend; grid on;
ylabel('vx (m/s)');
title('Velocity Estimation (not directly measured)');

subplot(2,1,2);
plot(t, true_hist(4,:), 'b', 'DisplayName', 'True vz');
hold on;
plot(t, est_hist(4,:), 'r', 'DisplayName', 'EKF vz');
legend; grid on;
ylabel('vz (m/s)');
xlabel('time (s)');

% -------------------------------------------------------
% Plot 4 — GPS dropout demonstration
% -------------------------------------------------------
figure;
subplot(2,1,1);
plot(t, true_hist(1,:), 'b', 'LineWidth', 2, ...
     'DisplayName', 'True px');
hold on;
plot(t, est_hist(1,:), 'r', 'LineWidth', 1.5, ...
     'DisplayName', 'EKF estimate');
xregion(dropout_t_start, dropout_t_end, ...
        'FaceColor', 'red', 'FaceAlpha', 0.1, ...
        'DisplayName', 'GPS dropout');
legend; grid on;
ylabel('x position (m)');
title('EKF Position Estimation with GPS Dropout');

subplot(2,1,2);
plot(t, true_hist(2,:), 'b', 'LineWidth', 2, ...
     'DisplayName', 'True pz');
hold on;
plot(t, est_hist(2,:), 'r', 'LineWidth', 1.5, ...
     'DisplayName', 'EKF estimate');
xregion(dropout_t_start, dropout_t_end, ...
        'FaceColor', 'red', 'FaceAlpha', 0.1, ...
        'DisplayName', 'GPS dropout');
legend; grid on;
ylabel('z position (m)');
xlabel('time (s)');