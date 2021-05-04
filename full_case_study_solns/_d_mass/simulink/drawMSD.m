function drawMSD(u, P)

    % process inputs to function
    z        = u(1);
    %zdot     = u(2);
    t        = u(3);
    
    % define persistent variables 
    persistent mass_handle
    persistent spring_handle
    
    % first time function is called, initialize plot and persistent 
    % vars
    if t==0
        figure(1), clf
        plot([-P.length-P.length/5,2*P.length],[0,0],'k--'); % plot track
        hold on
        plot([-P.length, -P.length], [0, 2*P.width],'k'); % plot wall
        mass_handle = drawMass(z, P.width, P.length, []);
        spring_handle = drawSpring(z, P.width, P.length, []);
        axis([-P.length-P.length/5, 2*P.length, -P.length, 2*P.length]);
        
    % at every other time step, redraw base and rod
    else 
        drawMass(z, P.width, P.length, mass_handle);
        drawSpring(z, P.width, P.length, spring_handle);
        drawnow
    end
end

   
%
%==================================================================
% drawMass
% draw the mass
% return handle if 3rd argument is empty, otherwise use 3rd arg as 
% handle
%==================================================================
function handle = drawMass(z, w, L, handle)
  
  X = [z-w/2, z+w/2, z+w/2, z-w/2];
  Y = [0, 0, w, w];
  
  if isempty(handle),
    handle = fill(X,Y,'b');
  else
    set(handle,'XData',X,'YData',Y);
  end
end
 
%
%===================================================================
% drawSpring
% draw the cord
% return handle if 3rd argument is empty, otherwise use 3rd arg as 
% handle
%===================================================================
function handle = drawSpring(z, w, L, handle)

  X = [-L, z-w/2];
  Y = [w/2, w/2];

  if isempty(handle),
    handle = plot(X, Y, 'g');
  else
    set(handle,'XData',X,'YData',Y);
  end
end

  