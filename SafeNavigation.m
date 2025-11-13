% --- File: TaskAltitude.m (con attivazione "smooth") ---

classdef SafeNavigation < Task   
    properties
        % Questo è il "muro" di sicurezza, la profondità a cui A=1
        max_depth = -38.0;
        
        % Parametri
        Kp = 1.0; % Guadagno per "spingere via"
        
        % Definisce la "zona di transizione" per lo smooth
        % La transizione avverrà tra -39.0 e (-39.0 + 0.5 = -38.5)
        transition_range = 0.5; 
    end
    
    methods
        % --- Costruttore ---
        function obj = TaskAltitude(depth_limit)
            % super@Task(); % Eredita dalla classe base
            if nargin > 0
                obj.max_depth = depth_limit;
            end
        end

        % --- 1. Riferimento (xdotbar) ---
        % (Scalare 1x1: velocità z desiderata)
        
        function updateReference(obj, robot)
            current_z = robot.eta(3); %
            error_z = obj.max_depth - current_z;
            
            if error_z > 0
                obj.xdotbar = obj.Kp * error_z;
            else
                obj.xdotbar = 0; % Sei al sicuro
            end
        end
        
        % --- 2. Jacobiano (J) ---
        % (Matrice 1x13: seleziona 'w')
        % (Questo è corretto, non va cambiato)
        function updateJacobian(obj, robot)
            obj.J = zeros(1, 13);
            obj.J(1, 10) = 1; 
        end
        
        % --- 3. Attivazione (A) ---
        % (Scalare 1x1: logica "smooth")
        function updateActivation(obj, robot)
            % Prende la posizione z attuale
            current_z = robot.eta(3); %

            % Definiamo i limiti per la funzione smooth
            
            % xmin: sotto questo valore, A = 1 (pericolo)
            xmin = obj.max_depth; % es: -39.0
            
            % xmax: sopra questo valore, A = 0 (sicuro)
            xmax = obj.max_depth + obj.transition_range; % es: -39.0 + 0.5 = -38.5
            
            % ymin: attivazione minima (0)
            ymin = 0;
            
            % ymax: attivazione massima (1)
            ymax = 1;
            
            % Calcola l'attivazione "smooth"
            % Se current_z = -40 (sotto xmin), A = ymax = 1
            % Se current_z = -38 (sopra xmax), A = ymin = 0
            % Se current_z = -38.7 (in mezzo), A sarà es. 0.6
            % obj.A = DecreasingBellShapedFunction(xmin, xmax, ymin, ymax, current_z);
            obj.A = IncreasingBellShapedFunction(xmin, xmax, ymin, ymax, current_z);
            %
        end
    end
end