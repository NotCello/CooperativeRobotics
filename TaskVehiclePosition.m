classdef TaskVehiclePosition < Task
    % Definisce il task per il controllo della POSIZIONE del veicolo.
    % Questo task controlla direttamente la velocità lineare [u, v, w]
    % nel frame veicolo.
    
    properties
        % Nessuna proprietà aggiuntiva
    end
    
    methods
        function updateReference(obj, robot)
            % Questo corrisponde ai passi 6 e 7 della foto.
            % Calcoliamo il riferimento x_dot = lambda * d,
            % ma poi lo proiettiamo nel frame del veicolo.
            
            % 1. Calcola l'errore 'd' nel frame MONDO (come da passo 1: Og - Ov)
            current_pos = robot.eta(1:3);
            goal_pos = robot.vehicleGoalPosition; %
            error_pos_world = goal_pos - current_pos;
            
            % 3. Il riferimento è una velocità lineare [u, v, w] desiderata
            %    (corrisponde a x_dot = lambda * d)
            Kp = 0.2; % Questo è il nostro guadagno 'lambda'
            obj.xdotbar = Kp * error_pos_world;
            
            % 4. Satura la velocità per sicurezza
            %obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        
        function updateJacobian(obj, robot)
            % Questo corrisponde al passo 5, ma semplificato.
            % Il nostro task 'x' è la velocità lineare [u, v, w].
            % Il nostro vettore di controllo 'y' è [q_dot(7); v_nu(6)].
            % v_nu = [u, v, w, p, q, r]'
            % Vogliamo selezionare [u, v, w] da v_nu.
            
            % 1. Parte del braccio (q_dot): [zeros(3, 7)]
            %    I giunti non influenzano [u, v, w]
            J_arm = zeros(3, 7);
            
            % 2. Parte del veicolo (v_nu): [eye(3), zeros(3, 3)]
            %    Seleziona [u, v, w] e ignora [p, q, r]
            R_v_w = robot.vTw(1:3, 1:3); % Matrice di rotazione da mondo a veicolo

            J_vehicle = [R_v_w', zeros(3, 3)];
            
            % 3. Combina le parti
            obj.J = [J_arm, J_vehicle]; % Dimensione finale: [3 x 13]
        end
        
        function updateActivation(obj, robot)
            % Questo corrisponde al passo 8.
            % È un "equality task", quindi l'attivazione è l'identità.
            obj.A = eye(3);
        end
    end
end