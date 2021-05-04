classdef ballbeamAnimation
    %
    %    Ballbeam animation
    %
    %--------------------------------
    properties
        ball_handle
        beam_handle
        length
        radius
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = ballbeamAnimation(P)
            self.length = P.length;
            self.radius = P.radius;
            
            figure(1), clf
            plot([0,P.length],[0,0],'k'); 
            hold on
            % initialize the ball and beam to initial conditions
            self=self.drawBall(P.z0, P.theta0);
            self=self.drawBeam(P.theta0);
            axis([-P.length/5, P.length+P.length/5, -P.length, P.length]);
        end
        %---------------------------
        function self=drawBallbeam(self, x)
            % Draw ballbeam is the main function that will call the functions:
            % drawBall and drawBeam to create the animation.
            % x is the system state
            z = x(1);       % Horizontal position of ball, m
            theta = x(2);   % Angle of beam, rads

            self=self.drawBall(z, theta);
            self=self.drawBeam(theta);
            drawnow
        end
        %---------------------------
        function self=drawBall(self, z, theta)
            N = 20;
            xi = 0:(2*pi/N):2*pi;
            X = z*cos(theta) - self.radius*sin(theta) + self.radius*cos(xi);
            Y = z*sin(theta) + self.radius*cos(theta) + self.radius*sin(xi);

            if isempty(self.ball_handle)
                self.ball_handle = fill(X, Y, 'g');
            else
                set(self.ball_handle, 'XData', X, 'YData', Y);
            end
        end 
        %---------------------------
        function self=drawBeam(self, theta)
            X = [0, self.length*cos(theta)];
            Y = [0, self.length*sin(theta)];

            if isempty(self.beam_handle)
                self.beam_handle = plot(X, Y, 'g', 'Linewidth', 2);
            else
                set(self.beam_handle,'XData', X, 'YData', Y);
            end
        end
    end
end