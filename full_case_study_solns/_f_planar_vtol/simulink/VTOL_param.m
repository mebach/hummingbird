%clear all

% Physical parameters of the VTOL known to the controller
P.mc = 1.0;  % kg
P.mr = 0.25;     % kg
P.Jc = 0.0042; %kg m^2
P.d = 0.3; % m
P.mu = 0.1; % kg/s
P.g = 9.81; % m/s^2

% Initial Conditions
P.z0 = 0.0;        % initial lateral position
P.h0 = 0.0;          % initial altitude
P.theta0 = 0;      % initial roll angle
P.zdot0 = 0;       % initial lateral velocity
P.hdot0 = 0;       % initial climb rate
P.thetadot0 = 0;   % initial roll rate

% Simulation Parameters
P.t_start = 0.0;  % Start time of simulation
P.t_end = 50.0;   % End time of simulation
P.Ts = 0.01;      % sample time for simulation
P.t_plot = 0.1;   % the plotting and animation is updated at this rate


% equilibrium force
P.Fe = ((P.mc+2*P.mr)*P.g);

% mixing matrix
P.mixing = inv([1, 1; P.d, -P.d]);

% maximum force on each rotor
P.fmax = 10; % N


