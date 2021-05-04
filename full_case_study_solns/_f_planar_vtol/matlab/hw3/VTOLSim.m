VTOLParamHWB;  % load parameters

% instantiate VTOL, and reference input classes 
VTOL = VTOLDynamics(P);  
addpath('../hw2'); z_reference = signalGenerator(0.5, 0.02);  
addpath('../hw2'); h_reference = signalGenerator(0.5, 0.02);
addpath('../hw2'); force = signalGenerator(0.001, 1);
addpath('../hw2'); torque = signalGenerator(0.001, 1);

% instantiate the data plots and animation
addpath('../hw2'); dataPlot = dataPlotter(P);
addpath('../hw2'); animation = VTOLAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        z_ref = z_reference.square(t);
        h_ref = h_reference.sin(t);
        f = P.Fe + force.sin(t);  % Calculate the input force
        tau = torque.sin(t); % Input torque
        u = P.mixing*[f; tau];
        VTOL.update(u);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.update(VTOL.state, z_ref);
    dataPlot.update(t, VTOL.state, z_ref, h_ref, f, tau);
end
