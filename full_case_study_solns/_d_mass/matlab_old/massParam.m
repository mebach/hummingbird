clear all

% Physical parameters known to the controller
P.m = 5;  % kg
P.k = 3;  % Kg/s^2
P.b = 0.5; % Kg/s

% parameters for animation
P.length = 5;
P.width = 1.0;

% initial conditions
P.z0 = 0;
P.zdot0 = 0;

% Simulation Parameters
P.t_start = 0.0;  % Start time of simulation
P.t_end = 50.0;   % End time of simulation
P.Ts = 0.01;      % sample time for simulation
P.t_plot = 0.1;   % the plotting and animation is updated at this rate

% dirty derivative parameters
P.sigma = 0.05; % cutoff freq for dirty derivative
P.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts); % dirty derivative gain

% input constraint
P.F_max = 2;
