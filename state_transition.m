function x_new = state_transition(x, u, dt, m, g)
% state_transition.m
% Propagates drone state forward one timestep using physics.
%
% Inputs:
%   x  = current state [px, pz, vx, vz, theta] (5x1)
%   u  = control input [T] thrust in Newtons (1x1)
%   dt = timestep (s)
%   m  = mass (kg)
%   g  = gravity (m/s^2)
%
% Output:
%   x_new = next state [px, pz, vx, vz, theta] (5x1)

% Unpack state
px    = x(1);
pz    = x(2);
vx    = x(3);
vz    = x(4);
theta = x(5);

% Unpack control
T = u(1);

% Accelerations from thrust and gravity
ax = (T/m) * sin(theta);
az = (T/m) * cos(theta) - g;

% Propagate state forward
x_new = [
    px + vx*dt;
    pz + vz*dt;
    vx + ax*dt;
    vz + az*dt;
    theta
    ];

end