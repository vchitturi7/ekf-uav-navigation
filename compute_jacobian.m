function F = compute_jacobian(x, u, dt, m)
% compute_jacobian.m
% Computes the Jacobian of the state transition function.
% This is the F matrix used in the EKF predict step.
%
% Inputs:
%   x  = current state [px, pz, vx, vz, theta] (5x1)
%   u  = control input [T] thrust in Newtons (1x1)
%   dt = timestep (s)
%   m  = mass (kg)
%
% Output:
%   F = 5x5 Jacobian matrix

% Unpack state
theta = x(5);

% Unpack control
T = u(1);

% Precompute repeated terms
T_over_m = T / m;

% Jacobian matrix
% F(i,j) = partial derivative of equation i with respect to state j
% States: [px, pz, vx, vz, theta]
F = [1,  0,  dt, 0,  0;
    0,  1,  0,  dt, 0;
    0,  0,  1,  0,  T_over_m * cos(theta) * dt;
    0,  0,  0,  1, -T_over_m * sin(theta) * dt;
    0,  0,  0,  0,  1];

end