classdef VTOLAnimation < handle
    %
    %    VTOL animation
    %
    %--------------------------------
    properties
        vehicle_handle
        target_handle
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = VTOLAnimation(P)
            figure(1), clf
            plot([0,P.length],[0,0],'k'); % plot track
            hold on
            % initialize the VTOL and target to initial conditions
            self=self.drawVehicle(P.z0, P.h0, P.theta0);
            self=self.drawTarget(P.target0);
            axis([-P.length/5, P.length+P.length/5, -P.length, P.length]);
        end
        %---------------------------
        function self=drawVTOL(self, x, target)
            z = x(1);
            h = x(2);
            theta = x(3);
            self=self.drawVehicle(z, h, theta);
            self=self.drawTarget(target);
            drawnow
        end
        %---------------------------
        function self=drawVehicle(self, z, h, theta)
            % Draw VTOL air vehicle.
            x1 = 0.1;
            x2 = 0.3;
            x3 = 0.4;
            y1 = 0.05;
            y2 = 0.01;
            pts = [...
                x1, y1;...
                x1, 0;...
                x2, 0;...
                x2, y2;...
                x3, y2;...
                x3, -y2;...
                x2, -y2;...
                x2, 0;...
                x1, 0;...
                x1, -y1;...
                -x1, -y1;...
                -x1, 0;...
                -x2, 0;...
                -x2, -y2;...
                -x3, -y2;...
                -x3, y2;...
                -x2, y2;...
                -x2, 0;...
                -x1, 0;...
                -x1, y1;...
                x1, y1;...
                ];
            % rotate points (must do first) 
            R = [cos(theta), sin(theta); -sin(theta), cos(theta)];
            pts = pts*R;
            % translate points
            pts = pts + repmat([z,h],size(pts,1),1);
  
            if isempty(self.vehicle_handle)
                self.vehicle_handle = fill(pts(:,1),pts(:,2),'b');
            else
                set(self.vehicle_handle,'XData',pts(:,1),'YData',pts(:,2));
            end
        end
        %---------------------------
        function self=drawTarget(self, target)
            w = 0.1;
            h = 0.05;
            pts = [...
                w/2, h;...
                w/2, 0;...
                -w/2, 0;...
                -w/2, h;...
                w/2, h;...
                ];
  
            % translate points
            pts = pts + repmat([target,0],size(pts,1),1);

            if isempty(self.target_handle)
                self.target_handle = fill(pts(:,1), pts(:,2), 'r');
            else
                set(self.target_handle,'XData',pts(:,1),'YData',pts(:,2));
            end
        end 
    end
end