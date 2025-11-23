classdef TaskVehiclePose< Task
    properties
        gain = 0.5    % proportional gain to reference velocity
    end
    
    methods
        
        function updateReference(obj, robot)
            % 1. Get the 6-DOF error
            [v_ang, v_lin] = CartError(robot.wTgv , robot.wTv);
    
            % 2. Create the desired velocity vector
            %    The order MUST be [linear; angular] to match v_nu
            obj.xdotbar = obj.gain * [v_lin; v_ang]; 
        end

        function updateJacobian(obj, robot)
            % Selects the 6 vehicle velocities (v_nu)
            obj.J = [zeros(6,7), eye(6)];
        end
        
        function updateActivation(obj, robot)
            % 1. Get the linear error to check distance
            [~, v_lin] = CartError(robot.wTgv, robot.wTv);
            error_norm = norm(v_lin);
    
            % 2. Calculate the "dimmer" value (0 to 1)
            activation_scalar = IncreasingBellShapedFunction(0.1, 0.5, 0, 1, error_norm);
    
            % 3. Create the 6x6 "A" matrix
            obj.A = activation_scalar * eye(6);
        end    
    end
end