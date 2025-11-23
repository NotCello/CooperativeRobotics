% Add paths
addpath('./simulation_scripts');
addpath('./tools');
addpath('./icat');
addpath('./robust_robot');
clc; clear; close all;

% Simulation parameters
dt       = 0.005;
endTime  = 50;
% Initialize robot model and simulator
robotModel = UvmsModel();         
sim = UvmsSim(dt, robotModel, endTime);
% Initialize Unity interface
unity = UnityInterface("127.0.0.1");

% --- 1. Create ALL tasks (your "Lego bricks") ---
task_safety_wall = TaskAltitude(); % Safety task
task_keep_level  = TaskHorizontal();      % Attitude task
task_nav_6dof    = TaskVehiclePose();     % New 6-DOF task
task_land_z      = TaskGoToAltitude(); % New landing task
task_nav_3dof    = TaskVehiclePosition();         % Existing 3-DOF task
task_tool       = TaskTool();

% --- 2. Create the "Master List" ---
all_tasks = {task_safety_wall, task_keep_level, task_nav_6dof, task_land_z, task_nav_3dof};

% --- 3. Configure ActionManager ---
actionManager = ActionManager(); %
actionManager.setUnifiedTaskList(all_tasks);

% --- 4. Define your two Actions ---
actionManager.addAction("safe_navigation", ...
    {task_safety_wall,task_keep_level ,task_nav_6dof});

actionManager.addAction("landing", ...
    {task_keep_level, task_land_z, task_nav_3dof});

% --- 5. Set Initial Position and Goal ---
% As per prompt: start close to seafloor
robotModel.eta    = [48.5 11.5 -33 0 0 0]'; 
robotModel.updateTransformations();

% Set goal for "safe navigation"
goal_nav = [50 -12.5 -33]';
% --- Line 45 (NEW) ---
robotModel.setGoal([0;0;0], [0;0;0], goal_nav, [0,0,0]);

% --- 6. Set Initial Action ---
actionManager.setCurrentAction("safe_navigation", 0.0);

% Initialize the logger
logger = SimulationLogger(ceil(endTime/dt)+1, robotModel, all_tasks); % Use the MASTER list

% --- 7. MAIN LOOP ---
for step = 1:sim.maxSteps
    % 1. Receive altitude
    robotModel.altitude = unity.receiveAltitude(robotModel);
    


    % --- 8. TRIGGER THE TRANSITION ---
    if (norm(robotModel.eta(1:3) - goal_nav(1:3)) < 1.0 && ... % If we are < 1m from nav goal
        ~strcmp(actionManager.currentActionName, "landing")) % And not already landing

        fprintf('--- ACTION SWITCH: INITIATING LANDING --- \n');
        
        % Set the new goal (land at current X,Y)
        land_pos = [robotModel.eta(1); robotModel.eta(2); -34]; % Land at -34
        % --- Line 45 (NEW) ---
        % --- Line 68 (NEW) ---
        robotModel.setGoal([0;0;0], [0;0;0], land_pos, [0,0,0]);
        
        % Call the new action
        actionManager.setCurrentAction("landing", sim.time);
    end

    % 2. Compute control (pass sim.time)
    [v_nu, q_dot] = actionManager.computeICAT(robotModel, sim.time);

    % 3. Step, 4. Send, 5. Log
    sim.step(v_nu, q_dot);
    unity.send(robotModel);
    logger.update(sim.time, sim.loopCounter);

    % 6. Optional debug prints
    if mod(sim.loopCounter, round(1 / sim.dt)) == 0
        fprintf('t = %.2f s\n', sim.time);
        fprintf('position  = %.2f \n',robotModel.eta) % Print pose
    end

    % 7. Optional real-time slowdown
    SlowdownToRealtime(dt);
end

% Display plots
logger.plotAll();

% Clean up Unity interface
delete(unity);