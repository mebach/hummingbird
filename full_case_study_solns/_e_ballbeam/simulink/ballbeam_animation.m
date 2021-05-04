function ballbeam_animation(u, P)

    % process inputs to function
    z        = u(1);
    theta    = u(2);
    %zdot     = u(3);
    %thetadot = u(4);
    t        = u(5);
        
    % define persistent variables 
    persistent ball_handle
    persistent beam_handle
    
    % first time function is called, initialize plot and persistent 
    % vars
    if t==0
        figure(1), clf
        plot([0,P.length],[0,0],'k'); % plot track
        hold on
        ball_handle = drawBall(z, theta, P.radius, []);
        beam_handle = drawBeam(theta, P.length, []);
        axis([-P.length/5, P.length+P.length/5, -0.7*P.length, 0.7*P.length]);
        axis('square');
    
        
    % at every other time step, redraw base and rod
    else 
        drawBall(z, theta, P.radius, ball_handle);
        drawBeam(theta, P.length, beam_handle);
    end
end

   
%
%=================================================================
% drawBall
% draw the ball
% return handle if 3rd argument is empty, otherwise use 3rd arg 
% as handle
%=================================================================
%
function handle = drawBall(z, theta, R, handle)
  
  N = 20;
  xi = 0:(2*pi/N):2*pi;
  X = z*cos(theta)-R*sin(theta)+R*cos(xi);
  Y = z*sin(theta)+R*cos(theta)+R*sin(xi);
  
  if isempty(handle)
    handle = fill(X,Y,'b');
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end
 
%
%=================================================================
% drawBeam
% draw the beam
% return handle if 3rd argument is empty, otherwise use 
% 3rd arg as handle
%=================================================================
%
function handle = drawBeam(theta, L,handle)

  
  X = [0, L*cos(theta)];
  Y = [0, L*sin(theta)];

  if isempty(handle)
    handle = plot(X, Y,'g','LineWidth',2);
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end

  