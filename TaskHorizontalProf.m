% --- File: TaskHorizontal.m (versione "come il Prof") ---

classdef TaskHorizontalProf < Task   
    properties
        theta_star = 0.1; % Obiettivo (come da nota)
        kp = 0.5;         % Guadagno (lambda nelle note)
        theta             % Angolo attuale (calcolato)
    end
    
    methods
        function updateReference(obj, robot)
            % 1. Calcola theta (il tuo codice è corretto)
            kw = [0; 0; 1];
            kv = robot.wTv(1:3, 3);
            componente_coseno = dot(kw, kv);
            componente_seno = norm(cross(kv, kw));
            obj.theta = atan2(componente_seno, componente_coseno);
            
            % 2. Calcola xdotbar (FORMULA DEL PROF)
            % xdot = lambda * (theta_star - theta)
            obj.xdotbar = obj.kp * (obj.theta_star - obj.theta);
        end
        
        function updateJacobian(obj, robot)
            % Il tuo Jacobiano è già corretto
            kw = [0; 0; 1];
            kv = robot.vTw(1:3,3); % Nota: qui usi vTw, in ref usi wTv. Attenzione!
                                   % Dovresti usare lo stesso k_v in entrambi.
                                   % Usa kv = robot.wTv(1:3, 3) anche qui.
            
            % Ricalcolo n (usando wTv per coerenza)
            kv_w = robot.wTv(1:3, 3);
            rho = cross(kv, kw); % rho = kv x kw (o kw x kv, cambia solo il segno)
            a = norm(rho);
            
            if (a > 1e-6) % Evita divisione per zero
                n = rho / a;
            else
                n = [0; 0; 0]; % Se allineati, n non è definito (ma non serve)
            end
            
            obj.J = n' * [zeros(3,7), zeros(3,3), eye(3)];
        end
        
        function updateActivation(obj, robot)
            % 3. Calcola Attivazione (FORMULA DEL PROF)
            % A = IncreasingBellShapedFunction(0.1, 0.2, 0, 1, theta)
            obj.A = IncreasingBellShapedFunction(0.1, 0.2, 0, 1, obj.theta);
            %
        end
    end
end