classdef ballbeamDynamics < handle
    %  Model the physical system
    %----------------------------
    properties
        state
        m1
        m2
        L
        g
        Ts
    end
    %----------------------------
    methods
        %---constructor-------------------------
        function self = ballbeamDynamics(P)
            % Initial state conditions
            self.state = [...
                        P.z0;...          % z initial position
                        P.theta0;...      % Theta initial orientation
                        P.zdot0;...       % zdot initial velocity
                        P.thetadot0;...   % Thetadot initial velocity
                        ];     
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % The parameters for any physical system are never known exactly.  Feedback
            % systems need to be designed to be robust to this uncertainty.  In the simulation
            % we model uncertainty by changing the physical parameters by a uniform random variable
            % that represents alpha*100 % of the parameter, i.e., alpha = 0.2, means that the parameter
            % may change by up to 20%.  A different parameter value is chosen every time the simulation
            % is run.
            alpha = P.alpha;  % Uncertainty parameter
            self.m1 = P.m1 * (1+2*alpha*rand-alpha);  
            self.m2 = P.m2 * (1+2*alpha*rand-alpha);  
            self.L = P.length * (1+2*alpha*rand-alpha);  
            self.g = P.g;  % the gravity constant is well known and so we don't change it.
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
            % re-label states and inputs for readability
            z = state(1);
            theta = state(2);
            zdot = state(3);
            thetadot = state(4);
            F = u;
            % The equations of motion.
            zddot     = (1/self.m1)*(self.m1*z*thetadot^2-self.m1*self.g*sin(theta));   

            thetaddot = (1/((self.m2*self.L^2)/3+self.m1*z^2))*...
                (-2*self.m1*z*zdot*thetadot-self.m1*self.g*z*cos(theta)...
                -self.m2*self.g*self.L/2*cos(theta)+self.L*F*cos(theta));

            % build xdot and return
            xdot = [zdot; thetadot; zddot; thetaddot];
        end
        %----------------------------
        function y = h(self)
            z = self.state(1);
            theta = self.state(2);
            y = [z; theta];
        end
    end
end


