% Add paths
addpath('./simulation_scripts');
addpath('./task');
addpath('./tools');
addpath('./icat');
addpath('./robust_robot');
clc; clear; close all;

% Simulation parameters
dt       = 0.005;
endTime  = 40;
% Initialize robot model and simulator
robotModel = UvmsModel();          
sim = UvmsSim(dt, robotModel, endTime);
% Initialize Unity interface
unity = UnityInterface("127.0.0.1");

% Define tasks     
task_tool    = TaskTool();
%task_set = {task_tool};

task_vehicle = TaskVehicle();  
task_alignment = TaskAlignment();
task_altitude = TaskAltitude();
task_zero_altitude = TaskZeroAltitude();

task_set_safe_navigation = {task_altitude, task_alignment, task_vehicle};
task_set_landing = {task_alignment, task_zero_altitude};

% Define actions and add to ActionManager
actionManager = ActionManager();
actionManager.addAction(task_set_safe_navigation, "safe_navigation");   % action 1
actionManager.addAction(task_set_landing, "landing");                   % action 2

% Unifying sets
all_sets = {task_set_safe_navigation, task_set_landing};
tasks = [all_sets{:}];
[~, ia] = unique(string(cellfun(@(t) t.id, tasks, 'UniformOutput', false)), 'stable');
unified_set = tasks(ia);
%unified_set = unique([all_sets{:}.id]);
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

trh = 0.2;
first = true;

% Main simulation loop
for step = 1:sim.maxSteps
    % 1. Receive altitude from Unity
    robotModel.altitude = unity.receiveAltitude(robotModel);

    %wdisp(robotModel.eta);

    [v_ang, v_lin] = CartError(robotModel.wTgv , robotModel.wTv);
    if norm(v_lin(1:2)) < trh & v_ang(3) < trh & first      % CONTROLLO ORIENTAMENTO SU Z PER EVITARE CONFLITTO
        actionManager.setCurrentAction("landing");
        first = false;
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