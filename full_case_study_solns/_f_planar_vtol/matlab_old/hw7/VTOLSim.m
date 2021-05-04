VTOLParamHW7;  % load parameters

% instantiate VTOL, controller, and reference input classes 
% Instantiate Dynamics class
VTOL = VTOLDynamics(P);  
ctrl = VTOLController(P);  
h_reference = signalGenerator(3, 0.02);  

% instantiate the data plots and animation
dataPlot = plotData(P);
animation = VTOLAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Get referenced inputs from signal generators
    h_ref = 5+h_reference.square(t);
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        u = ctrl.u([0; h_ref], VTOL.outputs());  % Calculate the control value
        VTOL.propagateDynamics(P.mixing*u);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.drawVTOL(VTOL.states, 0.0);
    dataPlot.updatePlots(t, VTOL.states, 0.0, h_ref, u(1), u(2));
end


