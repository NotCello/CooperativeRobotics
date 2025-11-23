classdef TaskGoToAltitude < Task
    properties
        
        kp = 0.5;
    end
    methods
        function updateReference(obj, robot)
            z_goal=robot.vehicleGoalPosition(3);
            error_z = z_goal - robot.eta(3); %
            obj.xdotbar = obj.kp * error_z;
        end
        function updateJacobian(obj, robot)
            obj.J = [zeros(1, 13)];
            obj.J(1, 10) = 1; % Controls 'w' velocity
        end
        function updateActivation(obj, robot)
            z_goal=robot.vehicleGoalPosition(3);
            error_z = z_goal - robot.eta(3);
            % Smooth 1-to-0 activation
            obj.A = IncreasingBellShapedFunction(0.05, 0.2, 0, 1, abs(error_z)); %
        end
    end
end