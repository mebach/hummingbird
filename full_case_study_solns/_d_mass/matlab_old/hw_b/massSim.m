massParamHWB;  % load parameters

% instantiate mass, and reference input classes 
mass = massDynamics(P);  
reference = signalGenerator(0.01, 0.02);  
force = signalGenerator(10, 0.05);

% instantiate the data plots and animation
dataPlot = plotData(P);
animation = massAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Get referenced inputs from signal generators
    ref_input = reference.square(t);
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        f = force.square(t);  % Calculate the input force
        mass.propagateDynamics(f);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.drawMass(mass.states);
    dataPlot.updatePlots(t, ref_input, mass.states, f);
end
