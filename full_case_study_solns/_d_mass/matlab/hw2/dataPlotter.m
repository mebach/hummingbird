classdef dataPlotter < handle
    %    This class plots the time histories for the pendulum data.
    %----------------------------
    properties
        % data histories
        time_history
        z_ref_history
        z_history
        force_history
        index
        % figure handles
        z_ref_handle
        z_handle
        force_handle
    end
    methods
        %--constructor--------------------------
        function self = dataPlotter(P)
            % Instantiate lists to hold the time and data histories
            self.time_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.z_ref_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.z_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.force_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.index = 1;

            % Create figure and axes handles
            figure(2), clf
            subplot(2, 1, 1)
                hold on
                self.z_ref_handle = plot(self.time_history, self.z_ref_history, 'g');
                self.z_handle    = plot(self.time_history, self.z_history, 'b');
                ylabel('z(m)')
                title('Mass Data')
            subplot(2, 1, 2)
                hold on
                self.force_handle    = plot(self.time_history, self.force_history, 'b');
                ylabel('force(N)')
        end
        %----------------------------
        function self = update(self, t, reference, states, ctrl)
            % update the time history of all plot variables
            self.time_history(self.index) = t;
            self.z_ref_history(self.index) = reference;
            self.z_history(self.index) = states(1);
            self.force_history(self.index) = ctrl;
            self.index = self.index + 1;

            % update the plots with associated histories
            set(self.z_ref_handle, 'Xdata', self.time_history, 'Ydata', self.z_ref_history)
            set(self.z_handle, 'Xdata', self.time_history, 'Ydata', self.z_history)
            set(self.force_handle, 'Xdata', self.time_history, 'Ydata', self.force_history)
        end
    end
end
