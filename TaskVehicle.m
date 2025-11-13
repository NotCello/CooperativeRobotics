classdef TaskVehicle < Task
    properties
        gain = 0.2    % proportional gain to reference velocity
    end

    methods

        function updateReference(obj, robot) % Compute desired reference velocity for the vehicle position task
            [v_ang, v_lin] = CartError(robot.wTgv , robot.wTv);% errore tra veicolo e goal_veicolo rispetto al mondo
            R = robot.vTw(1:3,1:3);
            obj.xdotbar = -obj.gain * R * v_lin;
            obj.xdotbar = Saturate(obj.xdotbar(1:3), 0.2);

 
            % if mod(round(robot.eta(1)*100), 10) == 0  % stampa ogni tanto
            %     disp(['Vehicle position: ', num2str(robot.eta(1:3)', '%.2f ')]);
            %     disp(['Reference velocity: ', num2str(obj.xdotbar', '%.3f ')]);
            % end
        end

        function updateJacobian(obj, robot)
            % for a task of cartesian position, J = I
            % Relationship between vehicle velocity (nu) and position rate in world
            % For a free-flying vehicle: only linear part affects position
            % So J = [I3  0], dimension (3x6)

            J_vehicle = [-eye(3) zeros(3,3)]; %riportato da frame veicolo al mondo robot.wTv(1:3,1:3)
            % The full system has 13 DOFs: [v_nu; q_dot]
            % So we append zeros for the manipulator part (7 joints)
            obj.J = [zeros(3,7) J_vehicle];
        end

        function updateActivation(obj, robot)
            % Activation function based on distance to goal 
            % x = robot.eta(1:3);
            % x_des = robot.vehicleGoalPosition;
            % 
            % % Norm of position error
            % err = norm(x_des - x); % act.f. depends on residual error ?
            % e_low = 0.01;
            % e_high = 0.1;
            % 
            % % ACTIVATION FUNCTION
            % if err <= e_low %task deactivation -- target reach
            %     alpha = 0;
            % elseif err >= e_high
            %     alpha = 1; % task active
            % else 
            %     alpha = 1 / (1 + exp(10 * (obj.x - obj.e_high))); % sigmoid
            % end

            obj.A = eye(3); % is an equality task = always active
        end

    end
end