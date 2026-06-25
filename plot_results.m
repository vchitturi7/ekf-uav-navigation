% plot_results.m
% Run this after simulation completes in Simulink
% Plots 3D trajectory, position error, and GPS dropout demonstration

close all;

% Extract data from Simulink output
true_state = squeeze(out.true_state)';   % 1001 x 8
ekf_state  = squeeze(out.ekf_state)';    % 1001 x 8
gps_meas   = squeeze(out.gps_meas)';     % 1001 x 3
t = linspace(0, 10, 1001)';

% -------------------------------------------------------
% Plot 1 — 3D position trajectory
% -------------------------------------------------------
figure;
plot3(true_state(:,1), true_state(:,2), true_state(:,3), ...
      'b', 'LineWidth', 2, 'DisplayName', 'True');
hold on;
plot3(ekf_state(:,1), ekf_state(:,2), ekf_state(:,3), ...
      'r', 'LineWidth', 1.5, 'DisplayName', 'EKF estimate');
plot3(gps_meas(:,1), gps_meas(:,2), gps_meas(:,3), ...
      'g.', 'MarkerSize', 4, 'DisplayName', 'GPS measurement');
legend; grid on;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title('3D EKF Drone Position Estimation');

% -------------------------------------------------------
% Plot 2 — position error over time
% -------------------------------------------------------
figure;
subplot(3,1,1);
plot(t, true_state(:,1) - ekf_state(:,1), 'r', 'DisplayName', 'EKF x error');
hold on;
plot(t, true_state(:,1) - gps_meas(:,1), 'g', 'DisplayName', 'GPS x error');
legend; grid on; ylabel('x error (m)');
title('Position Estimation Error Over Time');

subplot(3,1,2);
plot(t, true_state(:,2) - ekf_state(:,2), 'r', 'DisplayName', 'EKF y error');
hold on;
plot(t, true_state(:,2) - gps_meas(:,2), 'g', 'DisplayName', 'GPS y error');
legend; grid on; ylabel('y error (m)');

subplot(3,1,3);
plot(t, true_state(:,3) - ekf_state(:,3), 'r', 'DisplayName', 'EKF z error');
hold on;
plot(t, true_state(:,3) - gps_meas(:,3), 'g', 'DisplayName', 'GPS z error');
legend; grid on; ylabel('z error (m)'); xlabel('time (s)');

% -------------------------------------------------------
% Plot 3 — GPS dropout demonstration
% -------------------------------------------------------
figure;
subplot(3,1,1);
plot(t, true_state(:,1), 'b', 'LineWidth', 2, 'DisplayName', 'True px');
hold on;
plot(t, ekf_state(:,1), 'r', 'LineWidth', 1.5, 'DisplayName', 'EKF estimate');
xregion(3.0, 5.0, 'FaceColor', 'red', 'FaceAlpha', 0.1, 'DisplayName', 'GPS dropout');
legend; grid on; ylabel('x (m)');
title('EKF Position Estimation with GPS Dropout');

subplot(3,1,2);
plot(t, true_state(:,2), 'b', 'LineWidth', 2, 'DisplayName', 'True py');
hold on;
plot(t, ekf_state(:,2), 'r', 'LineWidth', 1.5, 'DisplayName', 'EKF estimate');
xregion(3.0, 5.0, 'FaceColor', 'red', 'FaceAlpha', 0.1, 'DisplayName', 'GPS dropout');
legend; grid on; ylabel('y (m)');

subplot(3,1,3);
plot(t, true_state(:,3), 'b', 'LineWidth', 2, 'DisplayName', 'True pz');
hold on;
plot(t, ekf_state(:,3), 'r', 'LineWidth', 1.5, 'DisplayName', 'EKF estimate');
xregion(3.0, 5.0, 'FaceColor', 'red', 'FaceAlpha', 0.1, 'DisplayName', 'GPS dropout');
legend; grid on; ylabel('z (m)'); xlabel('time (s)');

% -------------------------------------------------------
% Plot 4 — velocity estimation
% -------------------------------------------------------
figure;
subplot(3,1,1);
plot(t, true_state(:,4), 'b', 'DisplayName', 'True vx');
hold on;
plot(t, ekf_state(:,4), 'r', 'DisplayName', 'EKF vx');
legend; grid on; ylabel('vx (m/s)');
title('Velocity Estimation (not directly measured)');

subplot(3,1,2);
plot(t, true_state(:,5), 'b', 'DisplayName', 'True vy');
hold on;
plot(t, ekf_state(:,5), 'r', 'DisplayName', 'EKF vy');
legend; grid on; ylabel('vy (m/s)');

subplot(3,1,3);
plot(t, true_state(:,6), 'b', 'DisplayName', 'True vz');
hold on;
plot(t, ekf_state(:,6), 'r', 'DisplayName', 'EKF vz');
legend; grid on; ylabel('vz (m/s)'); xlabel('time (s)');