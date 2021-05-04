
% Physical parameters of the ballbeam known to the controller
P.m1 = 0.35;    % kg
P.m2 = 2;       % kg
P.length = 0.5; % m
P.g = 9.81;     % m/s^2

% parameters for animation
P.radius = 0.05;   % radius of ball

% Initial Conditions
P.z0 = P.length/2;         % initial ball position, m
P.theta0 = 0.0*pi/180;     % initial beam angle, rads
P.zdot0 = 0.0;             % initial ball velocity, m/s
P.thetadot0 = 0.0;         % initial beam angular velocity, rads/s

% Simulation Parameters
P.t_start = 0.0;  % Start time of simulation
P.t_end = 50.0;   % End time of simulation
P.Ts = 0.01;      % sample time for simulation
P.t_plot = 0.1;   % the plotting and animation is updated at this rate


% equilibrium force
P.Fe = P.m1*P.g*P.z0/P.length + P.m2*P.g/2;

% limits on force
P.Fmax = 15; % N
