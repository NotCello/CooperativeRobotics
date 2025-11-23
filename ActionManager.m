classdef ActionManager < handle
    properties
        actions = struct()      % cell array of actions (each action = stack of tasks)
        currentActionName = "" ;% name of currently active action
        previousActionName="";
        unifiedTaskList={};

        timeOfSwitch=-inf;
        transitionDuration=2.0;
    end

    methods
        function setUnifiedTaskList(obj, all_tasks)
        % Set the "master list" of all possible tasks
            obj.unifiedTaskList = all_tasks;
        end

        function addAction(obj,name, taskStack)
            % taskStack: cell array of tasks that define an action
            obj.actions.(name) = taskStack;
        end

        function [v_nu, qdot] = computeICAT(obj, robot, currentTime)
        
        % 1. Get Task Lists
        if isempty(obj.currentActionName)
            error('No action set. Call setCurrentAction first.');
        end
        tasks_new = obj.actions.(obj.currentActionName);
        % --- Line 30 (NEW) ---
        if strlength(obj.previousActionName) > 0
            tasks_old = obj.actions.(obj.previousActionName);
        else
            tasks_old = {}; % No previous action
        end

        % 2. Calculate Transition "Alpha" (0 to 1)
        elapsed = currentTime - obj.timeOfSwitch;
        alpha = min(elapsed / obj.transitionDuration, 1.0);

        % 3. Loop over the MASTER list
        for i = 1:length(obj.unifiedTaskList)
            task = obj.unifiedTaskList{i};
            
            % 4. Update Task's Internal State
            % (This order is CRITICAL for TaskHorizontal)
            task.updateReference(robot);
            task.updateActivation(robot);
            task.updateJacobian(robot);
            
            internal_A = task.A; % The activation the task *wants*

            % 5. Find what kind of task this is
            % --- Lines 54-55 (NEW) ---
            isInNew = any(cellfun(@(t) t == task, tasks_new));
            isInOld = any(cellfun(@(t) t == task, tasks_old));

            % 6. Set Final Activation (The Transition)
            if isInNew && ~isInOld
                task.A = internal_A * alpha; % FADE-IN
            elseif ~isInNew && isInOld
                task.A = internal_A * (1.0 - alpha); % FADE-OUT
            elseif isInNew && isInOld
                task.A = internal_A; % COMMON Task (stays on)
            else
                task.A = 0; % INACTIVE Task
            end
        end

        % 7. Perform ICAT (loops over MASTER list)
        ydotbar = zeros(13,1);
        Qp = eye(13);
        for i = 1:length(obj.unifiedTaskList)
            task = obj.unifiedTaskList{i};
            
            if (any(diag(task.A) > 1e-6)) % Only compute if task is active
                [Qp, ydotbar] = iCAT_task(task.A, task.J, ...
                                           Qp, ydotbar, task.xdotbar, ...
                                           1e-4, 0.01, 10);
            end
        end

        % 8. Final Damping and Split
        [~, ydotbar] = iCAT_task(eye(13), eye(13), Qp, ydotbar, zeros(13,1), 1e-4, 0.01, 10);
        qdot = ydotbar(1:7);
        v_nu = ydotbar(8:13);
    end

        function setCurrentAction(obj, name, currentTime)
        % Set the new action by name, save the old one
            if ~strcmp(obj.currentActionName, name)
                obj.previousActionName = obj.currentActionName;
                obj.currentActionName = name;
                obj.timeOfSwitch=currentTime;
            end
        end
    end
end