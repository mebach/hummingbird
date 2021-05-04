classdef VTOLDynamics < handle
    %  Model the physical system
    %----------------------------
    properties
        state
        mc
        mr
        Jc
        d
        mu
        g
        Ts
        F_wind         
    end
    %----------------------------
    methods
        %---constructor-------------------------
        function self = VTOLDynamics(P)
            % Initial state conditions
            self.state = [...
                        P.z0;...          % initial lateral position
                        P.h0;...          % initial altitude
                        P.theta0;...      % initial roll angle
                        P.zdot0;...       % initial lateral velocity
                        P.hdot0;...       % initial climb rate
                        P.thetadot0;...   % initial roll rate
                        ];     
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % The parameters for any physical system are never known exactly.  Feedback
            % systems need to be designed to be robust to this uncertainty.  In the simulation
            % we model uncertainty by changing the physical parameters by a uniform random variable
            % that represents alpha*100 % of the parameter, i.e., alpha = 0.2, means that the parameter
            % may change by up to 20%.  A different parameter value is chosen every time the simulation
            % is run.
            alpha = P.alpha;  % Uncertainty parameter
            self.mc = P.mc * (1+2*alpha*rand-alpha);
            self.mr = P.mr * (1+2*alpha*rand-alpha);  
            self.Jc = P.Jc * (1+2*alpha*rand-alpha);  
            self.d = P.d * (1+2*alpha*rand-alpha);  
            self.mu = P.mu * (1+2*alpha*rand-alpha);  
            self.g = P.g;  % the gravity constant is well known and so we don't change it.
            self.F_wind = P.F_wind * (1+2*alpha*rand-alpha); % wind disturbance force
            self.Ts = P.Ts; % sample rate at which dynamics is propagated
          
        end

        %----------------------------
        function y = update(self, u)
            self.rk4_step(u);
            y = self.h();
        end
        %----------------------------
        function self = rk1_step(self, u)
            %
            % Integrate the differential equations defining dynamics
            % P.Ts is the time step between function calls.
            % u contains the system input(s).
            % 
            % Integrate ODE using Runge-Kutta RK1 algorithm
            self.state = self.state + self.Ts * self.f(self.state, u);
        end
        %----------------------------
        function self = rk2_step(self, u)
            %
            % Integrate the differential equations defining dynamics
            % P.Ts is the time step between function calls.
            % u contains the system input(s).
            % 
            % Integrate ODE using Runge-Kutta RK2 algorithm
            F1 = self.f(self.state, u);
            F2 = self.f(self.state + self.Ts/2 * F1, u);
            self.state = self.state + self.Ts/6 * (F1 + F2);
        end
        %----------------------------
        function self = rk4_step(self, u)
            %
            % Integrate the differential equations defining dynamics
            % P.Ts is the time step between function calls.
            % u contains the system input(s).
            % 
            % Integrate ODE using Runge-Kutta RK4 algorithm
            F1 = self.f(self.state, u);
            F2 = self.f(self.state + self.Ts/2*F1, u);
            F3 = self.f(self.state + self.Ts/2*F2, u);
            F4 = self.f(self.state + self.Ts*F3, u);
            self.state = self.state + self.Ts/6 * (F1 + 2*F2 + 2*F3 + F4);
        end
        %----------------------------
        function xdot = f(self, state, u)
            %
            % Return xdot = f(x,u), the derivatives of the continuous states, as a matrix
            % 
            z = state(1);
            h = state(2);
            theta = state(3);
            zdot = state(4);
            hdot = state(5);
            thetadot = state(6);
            fr = u(1);
            fl = u(2);
            % The equations of motion.
            zddot     = (-(fr+fl)*sin(theta)-self.mu*zdot+self.F_wind)/(self.mc+2*self.mr);
            hddot     = (-(self.mc+2*self.mr)*self.g + (fr+fl)*cos(theta))/(self.mc+2*self.mr);
            thetaddot = self.d*(fr-fl)/(self.Jc+2*self.mr*self.d^2);

            % build xdot and return
            xdot = [zdot; hdot; thetadot; zddot; hddot; thetaddot];
        end
        %----------------------------
        function y = h(self)
            %
            % Returns the measured outputs as a list
            % [theta, phi] with added Gaussian noise
            % 
            % re-label states for readability
            % re-label states for readability
            z = self.state(1);
            h = self.state(2);
            theta = self.state(3);
            % add Gaussian noise to outputs
%             z_m = z + 0.001*randn;
%             h_m = h + 0.001*randn;
%             theta_m = theta + 0.001*randn;
            % return measured outputs
            y = [z; h; theta];
        end
    end
end


