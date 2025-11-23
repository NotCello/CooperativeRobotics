classdef TaskHorizontal < Task   
    properties
        theta_star = 0.1; % Goal angle (from notes)
        kp = 0.5;         % Gain (lambda from notes)
        
        % Properties to store calculated values
        theta = 0;        % Current error angle
        rho = [0;0;0];    % Misalignment vector (n*theta)
    end
    
    methods
        
        % --- 1. UPDATE REFERENCE (Calculates theta, rho, xdotbar) ---
        function updateReference(obj, robot)
            % Define the two vectors to compare
            k_w = [0; 0; 1];                    % World Z-axis (vertical)
            k_v = robot.wTv(1:3, 3);           % Vehicle Z-axis (in world frame)
            
            % --- Calculate Angle (theta) ---
            cos_theta = dot(k_w, k_v);
            sin_theta = norm(cross(k_w, k_v));
            obj.theta = atan2(sin_theta, cos_theta); % Always positive (0 to pi)
            
            % --- Calculate Misalignment Vector (rho) ---
            % Use the provided tool for this
            obj.rho = ReducedVersorLemma(k_v, k_w); %
            
            % --- Calculate Reference Rate (xdotbar) ---
            % Use the professor's formula
            obj.xdotbar = obj.kp * (obj.theta_star - obj.theta);
        end
        
        % --- 2. UPDATE JACOBIAN (Uses rho and theta) ---
        function updateJacobian(obj, robot)
            % Get the axis 'n' from 'rho' (which was calculated in updateReference)
            if obj.theta > 1e-6 % Avoid division by zero
                n = obj.rho / obj.theta;
            else
                n = [0; 0; 0]; % If aligned, axis is undefined (and J=0)
            end
            
            % J = n' * [selector for angular velocity]
            % (Matches professor's notes)
            obj.J = n' * [zeros(3,7), zeros(3,3), eye(3)];
        end
        
        % --- 3. UPDATE ACTIVATION (Uses theta) ---
        function updateActivation(obj, robot)
            % Use the "smooth" inequality function
            obj.A = IncreasingBellShapedFunction(0.1, 0.2, 0, 1, obj.theta);
            %
        end
    end
end