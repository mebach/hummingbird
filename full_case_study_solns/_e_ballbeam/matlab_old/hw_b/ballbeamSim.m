ballbeamParamHWB;  % load parameters

% instantiate pendulum, and reference input classes 
ballbeam = ballbeamDynamics(P);  
reference = signalGenerator(0.5, 0.02);  
force = signalGenerator(0.001, 1);

% instantiate the data plots and animation
dataPlot = plotData(P);
animation = ballbeamAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Get referenced inputs from signal generators
    ref_input = reference.square(t);
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        f = force.sin(t);  % Calculate the input force
        ballbeam.propagateDynamics(f);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.drawBallbeam(ballbeam.states);
    dataPlot.updatePlots(t, ref_input, ballbeam.states, f);
end
