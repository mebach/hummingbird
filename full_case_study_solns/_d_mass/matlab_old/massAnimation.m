classdef massAnimation < handle
    %
    %    Create mass animation
    %
    %--------------------------------
    properties
        weight_handle
        spring_handle
        length
        width
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = massAnimation(P)
            self.length = P.length;
            self.width = P.width;
            
            figure(1), clf
            plot([-P.length-P.length/5,2*P.length],[0,0],'k--'); % plot track
            hold on
            plot([-P.length, -P.length], [0, 2*P.width],'k'); % plot wall
            self.drawWeight(P.z0);
            self.drawSpring(P.z0);
            axis([-P.length-P.length/5, 2*P.length, -P.length, 2*P.length]);
        end
        %---------------------------
        function self=drawMass(self, x)
            z    = x(1); % position of mass, m
            %zdot = x(2); % velocity of mass

            self=self.drawWeight(z);
            self=self.drawSpring(z);
            drawnow
        end
        
        %---------------------------
        function self = drawWeight(self, x)
            z = x(1);
            X = [z-self.width/2, z+self.width/2, z+self.width/2, z-self.width/2];
            Y = [0, 0, self.width, self.width];
  
            if isempty(self.weight_handle)
                self.weight_handle = fill(X, Y, 'b');
            else
                set(self.weight_handle, 'XData', X, 'YData', Y);
                drawnow
            end
        end
        %---------------------------
        function self = drawSpring(self, x)
            z = x(1);
            X = [-self.length, z-self.width/2];
            Y = [self.width/2, self.width/2];
  
            if isempty(self.spring_handle)
                self.spring_handle = fill(X, Y, 'g');
            else
                set(self.spring_handle, 'XData', X, 'YData', Y);
                drawnow
            end
        end
    end
end