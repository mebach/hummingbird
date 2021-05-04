restoredefaultpath;
ballbeamParamHW10;  % load parameters

% instantiate ballbeam, controller, and reference input classes 
% Instantiate Dynamics class
ballbeam = ballbeamDynamics(P);  
ctrl = ballbeamController(P);  
    amplitude = 0.125; % amplitude of reference input
    frequency = 0.02; % frequency of reference input
reference = signalGenerator(amplitude, frequency);  

% instantiate the data plots and animation
dataPlot = plotData(P);
animation = ballbeamAnimation(P);

disturbance = 1.0;

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Get referenced inputs from signal generators
    ref_input = 0.25 + reference.square(t);
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        u = ctrl.u(ref_input, ballbeam.outputs());  % Calculate the control value
        input = u + disturbance;
        ballbeam.propagateDynamics(input);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.drawBallbeam(ballbeam.states);
    dataPlot.updatePlots(t, ref_input, ballbeam.states, u);
end


