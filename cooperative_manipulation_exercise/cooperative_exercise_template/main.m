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

%Initialize Cooperative Simulator Class
coop_system=coop_sim(dt,arm1,arm2,end_time);

%Define Object Shape and origin Frame
obj_length = 0.06;
w_obj_pos = [0.5 0 0.30]';
w_obj_ori = rotation(0,0,0);

%Set goal frames for left and right arm, based on object frame
%TO DO: Set arm goal frame based on object frame.
y_theta = pi/9;
arm1.setGoal(w_obj_pos,w_obj_ori,[w_obj_pos(1) - obj_length/2; w_obj_pos(2:3)],rotation(pi, -y_theta, 0));
arm2.setGoal(w_obj_pos,w_obj_ori,[w_obj_pos(1) + obj_length/2; w_obj_pos(2:3)],rotation(0, pi+y_theta, 0));

%Define Object goal frame (Cooperative Motion)
wTog=[rotation(0,0,0) [0.6, 0.4, 0.48]'; 0 0 0 1];
arm1.set_obj_goal(wTog)
arm2.set_obj_goal(wTog)

%Define Tasks, input values(Robot type(L,R,BM), Task Name)
left_tool_task=tool_task("toolLeft");
right_tool_task=tool_task("toolRight");

% minimum altitude
threshold_altitude = 0.15;
minimum_altitude_l = minimum_altitude_task("altitudeLeft", threshold_altitude);
minimum_altitude_r = minimum_altitude_task("altitudeRight", threshold_altitude);

% joint limits
joint_threshold = 0.2;
joint_limits_l = joint_limits_task("jointLimitsLeft", joint_threshold);
joint_limits_r = joint_limits_task("jointLimitsRight", joint_threshold);

% set cooperative velocitities
coop_vel_l = coop_movement_task("GraspCoopLeft");
coop_vel_r = coop_movement_task("GraspCoopRight");

% rigid grasp
rigid_move_l = rigid_move_task("rigidMoveL");
rigid_move_r = rigid_move_task("rigidMoveR");

% zero velocities
zero_velocities_l = zero_velocities_task("zeroVelocitiesLeft");
zero_velocities_r = zero_velocities_task("zeroVelocitiesRight");

%TO DO: Define the actions for each manipulator (remember the specific one
%for the cooperation)
go_to_left={joint_limits_l, minimum_altitude_l, left_tool_task};
go_to_right={joint_limits_r, minimum_altitude_r, right_tool_task};

coop_left={joint_limits_l, minimum_altitude_l, rigid_move_l};
coop_right={joint_limits_r, minimum_altitude_r, rigid_move_r};

zero_vel_left = {minimum_altitude_l, zero_velocities_l};
zero_vel_right = {minimum_altitude_r, zero_velocities_r};

%TO DO: Create two action manager objects to manage the tasks of a single
%manipulator (one for the non-cooperative and one for the cooperative steps
%of the algorithm)
%Load Action Manager Class and load actions
actionManager_arm1 = ActionManager();
actionManager_arm2 = ActionManager();

actionManager_arm1.addAction(go_to_left, "reachingLeft");
actionManager_arm1.addAction(coop_left, "graspingLeft");
actionManager_arm1.addAction(zero_vel_left, "zeroVelLeft");
actionManager_arm2.addAction(go_to_right, "reachingRight");
actionManager_arm2.addAction(coop_right, "graspingRight");
actionManager_arm2.addAction(zero_vel_right, "zeroVelRight");

% Unifying sets left
% all_sets_left = {go_to_left, coop_left, zero_vel_left};
% tasks_left = [all_sets_left{:}];
% [~, ia] = unique(string(cellfun(@(t) t.id, tasks_left, 'UniformOutput', false)), 'stable');
% unified_set_left = tasks_left(ia);
unified_set_left = {joint_limits_l, minimum_altitude_l, left_tool_task, rigid_move_l, zero_velocities_l};
unified_set_coop_left = [{coop_vel_l} unified_set_left(:)'];
actionManager_arm1.addUnifyingTasks(unified_set_left, unified_set_coop_left);


% Unifying sets right
% all_sets_right = {go_to_right, coop_right, zero_vel_right};
% tasks_right = [all_sets_right{:}];
% [~, ia] = unique(string(cellfun(@(t) t.id, tasks_right, 'UniformOutput', false)), 'stable');
% unified_set_right = tasks_right(ia);
unified_set_right = {joint_limits_r, minimum_altitude_r, right_tool_task, rigid_move_r, zero_velocities_r};
unified_set_coop_right = [{coop_vel_r} unified_set_right(:)'];
actionManager_arm2.addUnifyingTasks(unified_set_right, unified_set_coop_right);

% set current action left
actionManager_arm1.setCurrentAction("reachingLeft"); 
% set current action right
actionManager_arm2.setCurrentAction("reachingRight"); 

%Initiliaze robot interface
robot_udp=UDP_interface(real_robot);

% THRESHOLD REACH THE GOAL
threshold_goal = 1.0e-02;
% Initial bias for cooperation weights
mu0 = 0.001;
% rigid constraint
system_rigid_attached = false;

%Initialize logger
logger_left=SimulationLogger(ceil(end_time/dt)+1,coop_system.left_arm, actionManager_arm1);
logger_right=SimulationLogger(ceil(end_time/dt)+1,coop_system.right_arm, actionManager_arm2);
%Main simulation Loop
for t = 0:dt:end_time
    % switch cooperative actions
    switch_coop_actions = false;

    % 1. Receive UDP packets - DO NOT EDIT
    [ql,qr]=robot_udp.udp_receive(t);
    if real_robot==true %Only in real setup, assign current robot configuration as initial configuratio
        coop_system.left_arm.q=ql;
        coop_system.right_arm.q=qr;
    end
    % 2. Update Full kinematics of the bimanual system
    coop_system.update_full_kinematics();
    
    % 3. TO DO: compute the TPIK for each manipulator with your action
    % manager
    [ql_dot_nocoop]=actionManager_arm1.computeICAT(coop_system.left_arm, switch_coop_actions, dt);
    [qr_dot_nocoop]=actionManager_arm2.computeICAT(coop_system.right_arm, switch_coop_actions, dt);
    ql_dot = ql_dot_nocoop;
    qr_dot = qr_dot_nocoop;

    if (norm(coop_system.left_arm.dist_to_goal) < threshold_goal && ~coop_system.left_arm.grasped) && (norm(coop_system.right_arm.dist_to_goal) < threshold_goal && ~coop_system.right_arm.grasped)
        actionManager_arm1.setCurrentAction("graspingLeft");
        actionManager_arm2.setCurrentAction("graspingRight");
        coop_system.left_arm.grasped = true;
        coop_system.right_arm.grasped = true;

        system_rigid_attached = true;
        rigid_move_l.updateReference(coop_system.left_arm);
        rigid_move_r.updateReference(coop_system.right_arm);
    end
    if (norm(coop_system.left_arm.dist_to_goal) < threshold_goal && ~coop_system.left_arm.o_reached) && (norm(coop_system.right_arm.dist_to_goal) < threshold_goal && ~coop_system.right_arm.o_reached)
        actionManager_arm1.setCurrentAction("zeroVelLeft");
        actionManager_arm2.setCurrentAction("zeroVelRight");
        coop_system.left_arm.o_reached = true;
        coop_system.right_arm.o_reached = true;
    end
    if system_rigid_attached
        % RIGID BODY CONSTRAINT
        % Reference generator: desired object twist
        % Computed in Object Motion Task
        
        % 4. TO DO: COOPERATION hierarchy
        % SAVE THE NON COOPERATIVE VELOCITIES COMPUTED
        [v_ang, v_lin] = CartError(coop_system.left_arm.wTog, coop_system.left_arm.wTo);
        % Desired object velocity
        xdot_d = 1.0 * [v_ang; v_lin];

        % Desire tool velocity
        xdot_al = coop_system.left_arm.wJo * ql_dot_nocoop;
        xdot_ar = coop_system.right_arm.wJo * qr_dot_nocoop;

        % Compute weights        
        mu_a = mu0 + norm(xdot_d - xdot_al);
        mu_b = mu0 + norm(xdot_d - xdot_ar);

        % Weighted cooperative velocity (xhat_dot)
        x_hat_dot = (mu_a*xdot_al + mu_b*xdot_ar) / (mu_a + mu_b);

        % Compute rigid grasp constraint and feasible space
        Jo_l = coop_system.left_arm.wJo;
        Jo_r = coop_system.right_arm.wJo;
    
        % Compute the constraint matrices
        Ha = Jo_l*pinv(Jo_l);
        Hb = Jo_r*pinv(Jo_r);
        C = [Ha -Hb];
        Hab = [Ha zeros(6); zeros(6) Hb];

        % Project onto feasible subspace
        x_tilde_dot = Hab * (eye(12) - pinv(C)*C) * [x_hat_dot; x_hat_dot];

        % Set cooperative task reference
        coop_system.left_arm.xdot_coop_vel  = x_tilde_dot(1:6);
        coop_system.right_arm.xdot_coop_vel = x_tilde_dot(7:12);

        switch_coop_actions = true;

        % 5. TO DO: compute the TPIK for each manipulator with your action
        % manager (with the constrained action to track the coop velocity)
        [ql_dot] = actionManager_arm1.computeICAT(coop_system.left_arm, switch_coop_actions, dt);
        [qr_dot] = actionManager_arm2.computeICAT(coop_system.right_arm, switch_coop_actions, dt);
    end

    % 6. get the two variables for integration
    coop_system.sim(ql_dot,qr_dot);
    
    % 6. Send updated state to Pybullet
    robot_udp.send(t,coop_system)

    % 7. Loggging
    logger_left.update(coop_system.time,coop_system.loopCounter)
    logger_right.update(coop_system.time,coop_system.loopCounter)
    if mod(coop_system.loopCounter, round(1 / coop_system.dt)) == 0
        fprintf('Time: %.2f s ------ left (alt): %.2f m right (alt): %.2f\n', coop_system.time, coop_system.left_arm.wTt(3,4), coop_system.right_arm.wTt(3,4));
        if ~coop_system.left_arm.grasped
            fprintf('Left linear distance to goal: %.2f and angular: %.2f\n', norm(coop_system.left_arm.dist_to_goal), norm(coop_system.left_arm.rot_to_goal));
            fprintf('Right linear distance to goal: %.2f and angular: %.2f\n', norm(coop_system.right_arm.dist_to_goal), norm(coop_system.right_arm.rot_to_goal));
        elseif coop_system.left_arm.grasped && ~coop_system.left_arm.o_reached
            fprintf('Object linear distance to goal: %.2f and angular: %.2f\n', norm(coop_system.left_arm.dist_to_goal), norm(coop_system.left_arm.rot_to_goal));
            [v_ang, v_lin] = CartError(coop_system.left_arm.wTt , coop_system.right_arm.wTt);
            fprintf('Tool-Tool distance pose: %.2f\n', norm(v_lin))
            fprintf('Tool-Tool distance angle: %.2f\n', norm(v_ang));
        end
    end
    % 8. Optional real-time slowdown
    SlowdownToRealtime(dt);
end
%9. Display joint position, velocity and end effector velocities, Display for a given action, a number
%of tasks
% action=1;
% tasks=[1];
logger_left.plotAll();
logger_right.plotAll();

end
