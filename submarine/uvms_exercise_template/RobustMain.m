% Add paths
addpath('./simulation_scripts');
addpath('./task');
addpath('./tools');
addpath('./icat');
addpath('./robust_robot');
clc; clear; close all;

% Simulation parameters
dt       = 0.005;
endTime  = 85;
% Initialize robot model and simulator
robotModel = UvmsModel();          
sim = UvmsSim(dt, robotModel, endTime);
% Initialize Unity interface
unity = UnityInterface("127.0.0.1");

arm_length = 1.4; % m
% Define tasks     
task_tool    = TaskTool();

task_vehicle = TaskVehicle('safe_nav');
task_vehicle2 = TaskVehicle('landing');
task_alignment = TaskAlignment("alignment");
task_align_x = TaskAlign_x("align_x");
task_altitude = TaskAltitude("altitude", 2.0);
task_zero_altitude = TaskZeroAltitude("zero_altitude");
task_fixed_base = TaskFixedBase("fixed_base");
task_manipulability = TaskManipulability("manipulability", arm_length);

task_set_safe_navigation = {task_altitude, task_alignment, task_vehicle};
task_set_landing = {task_align_x, task_manipulability, task_zero_altitude};     % TASK PER NON FARLO STRISCIARE
task_set_grasp = {task_zero_altitude, task_fixed_base, task_tool};              % RIGURDARE TASK FIXED BASE PER ERRORE MOVIMENTO QUANDO IL GOAL È LONTANO

% Define actions and add to ActionManager
actionManager = ActionManager();
actionManager.addAction(task_set_safe_navigation, "safe_navigation");   % action 1
actionManager.addAction(task_set_landing, "landing");                   % action 2
actionManager.addAction(task_set_grasp, "grasping");                    % action 3

% Unifying sets
all_sets = {task_set_safe_navigation, task_set_landing, task_set_grasp};
tasks = [all_sets{:}];
[~, ia] = unique(string(cellfun(@(t) t.id, tasks, 'UniformOutput', false)), 'stable');
unified_set = tasks(ia);
actionManager.addUnifyingTasks(unified_set);

% set current action
actionManager.setCurrentAction("safe_navigation"); 

% Define desired positions and orientations (world frame)
w_arm_goal_position = [12.2025, 37.3748, -39.8860]';
w_arm_goal_orientation = [0, pi, pi/2];
%w_vehicle_goal_position = [12.2025   37.3748  -39.8860]';
%w_vehicle_goal_position = [10.5 		37.5	   -38]';
%w_vehicle_goal_position = [45 2 -33]';
w_vehicle_goal_position = [10.5 37.5 -38]';
w_vehicle_goal_orientation = [0, -0.06, 0.5];

% Set goals in the robot model
robotModel.setGoal(w_arm_goal_position, w_arm_goal_orientation, w_vehicle_goal_position, w_vehicle_goal_orientation);

% Initialize the logger
logger = SimulationLogger(ceil(endTime/dt)+1, robotModel, unified_set);

trh = 0.25;
first = false;
second = false;

% Main simulation loop
for step = 1:sim.maxSteps
    % 1. Receive altitude from Unity
    robotModel.altitude = unity.receiveAltitude(robotModel);

    %disp(robotModel.eta);

    [v_ang, v_lin] = CartError(robotModel.wTgv , robotModel.wTv);
    if norm(v_lin(1:2)) < trh && v_ang(3) < trh & ~first      % CONTROLLO ORIENTAMENTO SU Z PER EVITARE CONFLITTO
        actionManager.setCurrentAction("landing");
        first = true;
    end

    wTb = robotModel.wTv * robotModel.vTb;
    [~, v_lin] = CartError(robotModel.wTg , wTb);
    if ~isempty(robotModel.altitude) && robotModel.altitude <= 0.02 && ~second && first && norm(v_lin) <  arm_length * 0.8
        actionManager.setCurrentAction("grasping");
        second = true;
    end

    % 2. Compute control commands for current action
    [v_nu, q_dot] = actionManager.computeICAT(robotModel);

    % 3. Step the simulator (integrate velocities)
    sim.step(v_nu, q_dot);

    % 4. Send updated state to Unity
    unity.send(robotModel);

    % 5. Logging
    logger.update(sim.time, sim.loopCounter);

    % 6. Optional debug prints
    if mod(sim.loopCounter, round(1 / sim.dt)) == 0
        fprintf('t = %.2f s\n', sim.time);
        fprintf('alt = %.2f m\n', robotModel.altitude);
    end

    % 7. Optional real-time slowdown
    SlowdownToRealtime(dt);
end

% Display plots
logger.plotAll();

% Clean up Unity interface
delete(unity);


% function unified = mergeTaskSets(all_sets)
% % MERGETASKSETS Merge ordered task-sets into a unified ordered list.
% %   unified = mergeTaskSets(all_sets)
% %   all_sets: cell array where each element è una cell array ordinata di task
% %             (ogni task ha proprietà .id)
% %   unified: cell array ordinata, con possibili duplicati quando necessario
% 
% unified = {};                 % cell array di oggetti task
% unified_ids = strings(0);     % string ids corrispondenti (comodo per confronti)
% 
% for s = 1:numel(all_sets)
%     set = all_sets{s};
%     ids = string(cellfun(@(t) t.id, set, 'UniformOutput', false));
%     for k = 1:numel(set)
%         id = ids(k);
%         before = ids(1:max(0,k-1));
%         if k < numel(set)
%             after = ids(k+1:end);
%         else
%             after = strings(0);
%         end
% 
%         % trova tutte le occorrenze già presenti nella unified
%         same_idx = find(unified_ids == id);
%         found_consistent = false;
% 
%         % prova se una occorrenza esistente è consistente con l'ordine corrente
%         for idx = same_idx
%             ok_before = true;
%             for b = before
%                 b_idx = find(unified_ids == b);
%                 if ~isempty(b_idx) && max(b_idx) >= idx
%                     ok_before = false;
%                     break;
%                 end
%             end
%             if ~ok_before
%                 continue;
%             end
% 
%             ok_after = true;
%             for a = after
%                 a_idx = find(unified_ids == a);
%                 if ~isempty(a_idx) && min(a_idx) <= idx
%                     ok_after = false;
%                     break;
%                 end
%             end
%             if ok_after
%                 found_consistent = true;
%                 break;
%             end
%         end
% 
%         % se non esiste una occorrenza coerente, append (duplicazione)
%         if ~found_consistent
%             unified{end+1} = set{k};                %#ok<AGROW>
%             unified_ids(end+1) = id;               %#ok<AGROW>
%         end
%     end
% end
% end