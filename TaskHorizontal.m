classdef TaskHorizontal < Task   
    properties
        theta_star=0.1;
        kp=0.2;
        theta
    end
    methods
        function updateReference(obj, robot)
            k_w=[0 0 1]';
            k_v=robot.vTw(1:3,3);
            rho=skew(k_v)*k_w;
            a=norm(rho);
            % theta=ata
            kw = [0; 0; 1]; % Vettore Z verticale (asse del mondo)
            % L'asse Z del veicolo proiettato sul mondo.
            % Lo troviamo nella matrice di rotazione wTv.
            % (wTv è in robotModel)
            kv = robot.wTv(1:3, 3); 
            
            % 2. Calcola le due componenti
            x_component = dot(kw, kv);
            % y_component = norm(cross(kv, kw));
            
            % 3. Calcola l'angolo in radianti
            obj.theta = atan2(a, x_component);
            n=rho/obj.theta;
            
            
            obj.xdotbar = -0.2 * (obj.theta);
            % % limit the requested velocities...
            % obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.2);
            % obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.2);
            
            
            
            
        end
        function updateJacobian(obj, robot)
            % bJe = RobustJacobian(robot.q);
            % Ste = [eye(3) zeros(3);  -skew(robot.vTe(1:3,1:3)*robot.eTt(1:3,4)) eye(3)];
            % Jt_a  = Ste * [robot.vTb(1:3,1:3) zeros(3,3); zeros(3,3) robot.vTb(1:3,1:3)] * bJe;
            % Jt_v = [zeros(3) eye(3); eye(3) -skew(robot.vTt(1:3,4))];
            % obj.J = [Jt_a Jt_v];
            k_w=[0 0 1]';
            k_v=robot.vTw(1:3,3);
            rho=skew(k_v)*k_w;
            a=norm(rho);
            n=rho/a;
            obj.J=n'*[zeros(3,7),zeros(3,3),eye(3)];
        end
        
        function updateActivation(obj, robot)
            
            obj.A = IncreasingBellShapedFunction(0.1,0.2,0,1,obj.theta);
        end
    end
end