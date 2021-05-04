clear all

% Physical parameters known to the controller
P.m = 5;   % kg
P.k = 3;   % N/m
P.b = 0.5; % N-s/m

% parameters for animation
P.length = 5;
P.width = 1.0;

% initial conditions
P.z0 = 0;
P.zdot0 = 0;

% input constraint
P.F_max = 6;

% Simulation Parameters
P.t_start = 0.0;  % Start time of simulation
P.t_end = 100.0;   % End time of simulation
P.Ts = 0.01;      % sample time for simulation
P.t_plot = 0.1;   % the plotting and animation is updated at this rate


