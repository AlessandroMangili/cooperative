function main()
%Add path
addpath('./simulation_scripts');
addpath('./tools')
addpath('./icat')
addpath('./tasks')
clc;clear;close all; 
%Simulation Parameters
dt = 0.005;
end_time = 15;

% Initialize Franka Emika Panda Model
model = load("panda.mat");

%Simulation Setup
real_robot = false;

%Initiliaze panda_arm() Class, specifying the base offset w.r.t World Frame
arm1=panda_arm(model,eye(4));
%TO DO: TRANSFORMATION MATRIX FROM WORLD FRAME TO RIGHT ARM BASE FRAME
wTb2 = [
            -1  0   0   1.06;
            0   -1  0   -0.01;
            0   0   1   0
            0   0   0   1
        ];
arm2=panda_arm(model,wTb2);

%Initialize Bimanual Simulator Class
bm_sim=bimanual_sim(dt,arm1,arm2,end_time);

%Define Object Shape and origin Frame
obj_length = 0.10;
w_obj_pos = [0.5 0 0.59]';
w_obj_ori = rotation(0,0,0);

%Set goal frames for left and right arm, based on object frame
%TO DO: Set arm goal frame based on object frame.
y_theta = pi/6;
arm1.setGoal(w_obj_pos,w_obj_ori, [w_obj_pos(1) - obj_length/2; w_obj_pos(2:3)],rotation(pi, -y_theta, 0));
arm2.setGoal(w_obj_pos,w_obj_ori, [w_obj_pos(1) + obj_length/2; w_obj_pos(2:3)],rotation(0, pi+y_theta, 0));

%Define Object goal frame (Cooperative Motion)
wTog=[rotation(0,0,0) [0.65, -0.35, 0.28]'; 0 0 0 1];
arm1.set_obj_goal(wTog)
arm2.set_obj_goal(wTog)

%Define Tasks, input values(Robot type(L,R,BM), Task Name)
left_tool_task=tool_task("L","LT", "toolLeft");
right_tool_task=tool_task("R","RT", "toolRight");

% TASK
% minimum altitude
threshold_altitude = 0.15;
minimum_altitude_l = minimum_altitude_task("L", "LT", threshold_altitude, "altitudeLeft");
minimum_altitude_r = minimum_altitude_task("R", "RT", threshold_altitude, "altitudeRight");

% joint limits
joint_limits_l = joint_limits_task("L", "LT", "jointLimitsLeft");
joint_limits_r = joint_limits_task("R", "RT", "jointLimitsRight");

% rigid grasp
rigid_grasp_l = rigid_grasp_task("L","LT", "graspLeft");
rigid_grasp_r = rigid_grasp_task("R","RT", "graspRight");

% zero velocities
zero_velocities_l = zero_velocities_task("L", "LT", "zeroVelocitiesLeft");
zero_velocities_r = zero_velocities_task("R", "RT", "zeroVelocitiesRight");

%Load Action Manager Class and load actions
actionManager = ActionManager();

%Actions for each phase: go to phase, coop_motion phase, end_motion phase
go_to = {joint_limits_l,joint_limits_r,minimum_altitude_l, minimum_altitude_r, left_tool_task, right_tool_task};
coop = {joint_limits_l, joint_limits_r, minimum_altitude_l, minimum_altitude_r, rigid_grasp_l, rigid_grasp_r};
zero_vel = {minimum_altitude_l, minimum_altitude_r, zero_velocities_l, zero_velocities_r};

actionManager.addAction(go_to, "reaching");
actionManager.addAction(coop, "grasping");
actionManager.addAction(zero_vel, "zero_vel");

% Unifying sets
all_sets = {go_to, coop, zero_vel};
tasks = [all_sets{:}];
[~, ia] = unique(string(cellfun(@(t) t.id, tasks, 'UniformOutput', false)), 'stable');
unified_set = tasks(ia);
actionManager.addUnifyingTasks(unified_set);

% set current action
actionManager.setCurrentAction("reaching"); 

%Initiliaze robot interface
robot_udp=UDP_interface(real_robot);

%Initialize logger
logger=SimulationLogger(ceil(end_time/dt)+1,bm_sim,actionManager);

%Main simulation Loop
for t = 0:dt:end_time
    % 1. Receive UDP packets - DO NOT EDIT
    [ql,qr]=robot_udp.udp_receive(t);
    if real_robot==true %Only in real setup, assign current robot configuration as initial configuratio
        bm_sim.left_arm.q=ql;
        bm_sim.right_arm.q=qr;
    end
    % 2. Update Full kinematics of the bimanual system
    bm_sim.update_full_kinematics();
    
    % 3. Compute control commands for current action
    [q_dot]=actionManager.computeICAT(bm_sim);

    % 4. Step the simulator (integrate velocities)
    bm_sim.sim(q_dot);
    
    % 5. Send updated state to Pybullet
    robot_udp.send(t,bm_sim)

    %disp(arm1.q)

    % 6. Lggging
    logger.update(bm_sim.time,bm_sim.loopCounter)
    bm_sim.time
    % 7. Optional real-time slowdown
    SlowdownToRealtime(dt);

    if (norm(arm1.dist_to_goal) < 1.0e-03 && ~arm1.grasped) && (norm(arm2.dist_to_goal) < 1.0e-03 && ~arm2.grasped)
        actionManager.setCurrentAction("grasping"); 
        arm1.grasped = true;
        arm2.grasped = true;
        rigid_grasp_l.updateReference(bm_sim);
        rigid_grasp_r.updateReference(bm_sim);
    end

    if (norm(arm1.dist_to_goal) < 1.0e-03 && ~arm1.o_reached) && (norm(arm2.dist_to_goal) < 1.0e-03 && ~arm2.o_reached)
        actionManager.setCurrentAction("zero_vel"); 
        arm1.o_reached = true;
        arm2.o_reached = true;
    end
end
%Display joint position and velocity, Display for a given action, a number
%of tasks
action=1;
tasks=[1];
logger.plotAll(action,tasks);
end
