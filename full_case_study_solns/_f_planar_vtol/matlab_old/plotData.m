classdef plotData < handle
    %    This class plots the time histories for the VTOL data.
    %----------------------------
    properties
        % data histories
        time_history
        zref_history
        z_history
        href_history
        h_history
        theta_history
        force_history
        torque_history
        index
        % figure handles
        zref_handle
        z_handle
        href_handle
        h_handle
        theta_handle
        force_handle
        torque_handle
    end
    methods
        %--constructor--------------------------
        function self = plotData(P)
            % Instantiate lists to hold the time and data histories
            self.time_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.zref_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.z_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.href_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.h_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.theta_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.force_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.torque_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.index = 1;

            % Create figure and axes handles
            figure(2), clf
            subplot(5, 1, 1)
                hold on
                self.zref_handle = plot(self.time_history, self.zref_history, 'g');
                self.z_handle    = plot(self.time_history, self.z_history, 'b');
                ylabel('z(m)')
                title('VTOL Data')
            subplot(5, 1, 2)
                hold on
                self.href_handle = plot(self.time_history, self.href_history, 'g');
                self.h_handle    = plot(self.time_history, self.h_history, 'b');
                ylabel('h(m)')
            subplot(5, 1, 3)
                hold on
                self.theta_handle    = plot(self.time_history, self.theta_history, 'b');
                ylabel('theta(deg)')
            subplot(5, 1, 4)
                hold on
                self.force_handle    = plot(self.time_history, self.force_history, 'b');
                ylabel('force(N)')
            subplot(5, 1, 5)
                hold on
                self.torque_handle    = plot(self.time_history, self.torque_history, 'b');
                ylabel('torque(Nm)')
        end
        %----------------------------
        function self = updatePlots(self, t, states, z_ref, h_ref, force, torque)
            % update the time history of all plot variables
            self.time_history(self.index) = t;
            self.zref_history(self.index) = z_ref;
            self.z_history(self.index) = states(1);
            self.href_history(self.index) = h_ref;
            self.h_history(self.index) = states(2);
            self.theta_history(self.index) = 180/pi*states(3);
            self.force_history(self.index) = force;
            self.torque_history(self.index) = torque;
            self.index = self.index + 1;

            % update the plots with associated histories
            set(self.zref_handle, 'Xdata', self.time_history, 'Ydata', self.zref_history)
            set(self.z_handle, 'Xdata', self.time_history, 'Ydata', self.z_history)
            set(self.href_handle, 'Xdata', self.time_history, 'Ydata', self.href_history)
            set(self.h_handle, 'Xdata', self.time_history, 'Ydata', self.h_history)
            set(self.theta_handle, 'Xdata', self.time_history, 'Ydata', self.theta_history)
            set(self.force_handle, 'Xdata', self.time_history, 'Ydata', self.force_history)
            set(self.torque_handle, 'Xdata', self.time_history, 'Ydata', self.torque_history)
        end
    end
end
