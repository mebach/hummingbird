VTOLParamHWA  % load parameters

% instantiate reference input classes 
z_reference = signalGenerator(0.5, 0.1);
h_reference = signalGenerator(0.5, 0.1);
zRef = signalGenerator(0.5, 0.1);
hRef = signalGenerator(0.5, 0.1);
thetaRef = signalGenerator(2*pi, 0.1);   
fRef = signalGenerator(5, 0.5);
tauRef = signalGenerator(5, 0.5);


% instantiate the data plots and animation
dataPlot = plotData(P);
animation = VTOLAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % set variables
    z_r = z_reference.square(t);
    z = zRef.sin(t);
    h_r = h_reference.square(t);
    h = hRef.sin(t);
    theta = thetaRef.sin(t);
    f = fRef.sawtooth(t);
    tau = tauRef.sawtooth(t);
    % update animation and data plot
    state = [z; h; theta; 0.0; 0.0; 0.0];
    animation.drawVTOL(state, z_r);
    dataPlot.updatePlots(t, state, z_r, h_r, f, tau);
    t = t + P.t_plot;  % advance time by t_plot
    pause(0.1)
end


