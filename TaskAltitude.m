classdef TaskAltitude < Task   
    properties
        % Defines the safety limit relative to the seafloor
        min_altitude = 2.0;      % Minimum safe altitude (meters)
        kp = 2.0;                % Gain to push away from floor
        transition_range = 0.5;  % Activation buffer (starts acting at 1.5m)
    end
    
    methods
        function updateReference(obj, robot)
            % 1. Measure distance to seafloor
            current_altitude = robot.altitude;
            
            % 2. Calculate error (positive if we are too close)
            % We want altitude >= min_altitude
            dist_error = obj.min_altitude - current_altitude;
             
            % 3. Set reference velocity
            % If we are too close (dist_error > 0), command positive velocity (UP)
            if dist_error > 0
                 obj.xdotbar = obj.kp * dist_error;             
            else
                 % If we are safe, request 0 (or let lower priority tasks decide)
                 obj.xdotbar = 0; 
            end
        end
        
        function updateJacobian(obj, robot)
            % Control the vehicle linear z-velocity (w)
            % The Jacobian selects the 10th element of the state vector (v_nu(3))
            obj.J = zeros(1, 13);
            obj.J(1, 10) = 1; 
        end
        
        function updateActivation(obj, robot)
            % 1. Get current altitude
            current_altitude = robot.altitude;
            
            % 2. Define activation bounds
            % We want the task to be:
            % - Fully Active (1) if altitude <= 1.0m (Danger)
            % - Inactive (0) if altitude >= 1.5m (Safe)
            
            xmin = obj.min_altitude;                        % 1.0
            xmax = obj.min_altitude + obj.transition_range; % 1.5
            
            % 3. Compute activation
            % Use Decreasing function because lower altitude = higher danger
            obj.A = DecreasingBellShapedFunction(xmin, xmax, 0, 1, current_altitude);
        end
    end
end