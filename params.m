% params.m
% All simulation constants in one place.
% Run this at the start of run_simulation.m to load everything.

% Drone physical parameters
m = 1.0;       % mass (kg)
g = 9.81;      % gravity (m/s^2)

% Simulation
dt = 0.01;     % timestep (s)
N  = 1000;      % number of timesteps

% Initial true state [px, pz, vx, vz, theta]
x0 = [0; 10; 0; 0; 0];

% Measurement matrix — GPS measures px and pz only
H = [1 0 0 0 0;
    0 1 0 0 0];

% Process noise covariance Q
% How much you distrust your own dynamics model
% Larger = trust measurements more
% Reduce process noise — trust dynamics model more
Q = diag([0.001, 0.001, 0.01, 0.01, 0.0001]);

% Keep GPS noise the same
R = diag([2.0, 2.0]);

% Start with higher initial confidence
P0 = eye(5) * 0.01;

% Hover thrust — exactly cancels gravity
T_hover = m * g;