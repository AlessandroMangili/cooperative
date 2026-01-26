classdef ActionManager < handle
    properties
        actions = {}      % cell array of actions (each action = stack of tasks)
        action_names = []     % names of actions
        unifying_actions
        currentAction = 1 % index of currently active action
        previousAction = []
        actionSwitchTime = 0
        transitionDuration = 1.0
    end

    methods
        function addAction(obj, taskStack, action_name)
            % taskStack: cell array of tasks that define an action
            obj.actions{end+1} = taskStack;
            obj.action_names{end+1} = action_name;
        end

         function addUnifyingTasks(obj, unified_set)
            obj.unifying_actions = unified_set;
         end

        function setCurrentAction(obj, action_name)
            disp(['Seeking current action: ', action_name]);
            disp('Actions saved:');
            disp(obj.action_names);

            idx = find([obj.action_names{:}] == action_name, 1);
            if isempty(idx)
                error('Action "%s" not found.', action_name);
            end

            if idx ~= obj.currentAction
                obj.previousAction = obj.currentAction;
                obj.currentAction  = idx;
                obj.actionSwitchTime = 0;
            end
        end

        function [ydotbar] = computeICAT(obj,bm_system, dt)
            obj.actionSwitchTime = obj.actionSwitchTime + dt;

            tasks = obj.unifying_actions;
            task_ids = [];
            for i = 1:length(tasks)
                task_ids{i} = tasks{i}.id;
            end

            % current/previous task sets
            current_tasks = obj.actions{obj.currentAction};
            current_task_ids = [];
            for i = 1:length(current_tasks)
                current_task_ids{i} = current_tasks{i}.id;
            end

            prev_task_ids = [];
            if ~isempty(obj.previousAction)
                prev_tasks = obj.actions{obj.previousAction};
                for i = 1:length(prev_tasks)
                    prev_task_ids{i} = prev_tasks{i}.id;
                end
            else
                prev_tasks = {};
            end

            inCurrent = ismember(string(task_ids), string(current_task_ids));
            inPrev    = ismember(string(task_ids), string(prev_task_ids));

            for i = 1:length(tasks)
                task = tasks{i};
                task.updateReference(bm_system);
                task.updateJacobian(bm_system);
                task.updateActivation(bm_system);

                if (tasks{i}.id == "rigidMoveL" || tasks{i}.id == "rigidMoveR") && ~inCurrent(i)
                    alpha = 0;
                elseif (tasks{i}.id == "rigidMoveL" || tasks{i}.id == "rigidMoveR") && ~inPrev(i)
                    alpha = 1;
                elseif inCurrent(i) && ~inPrev(i)
                    % entering → fade in
                    alpha = IncreasingBellShapedFunction(0, obj.transitionDuration, 0, 1, obj.actionSwitchTime);
                elseif ~inCurrent(i) && inPrev(i)
                    % leaving → fade out
                    alpha = DecreasingBellShapedFunction(0, obj.transitionDuration, 0, 1, obj.actionSwitchTime);
                elseif ~inCurrent(i) && ~inPrev(i)
                    % steady → normal activation
                    alpha = 0;
                elseif inCurrent(i) && inPrev(i)
                    alpha = 1;
                end
                task.A = task.A * alpha;
            end
            % tasks = obj.actions{obj.currentAction};
            % %% 1. Update references, Jacobians, activations
            % for i = 1:length(tasks)
            %     tasks{i}.updateReference(bm_system);
            %     tasks{i}.updateJacobian(bm_system);
            %     tasks{i}.updateActivation(bm_system);
            % end
            
            %% 2. Perform ICAT (task-priority inverse kinematics) for the current Action
            ydotbar = zeros(14,1);
            Qp = eye(14);
            for i = 1:length(tasks)
                 %% TRANSITION FROM PREVIOUS ACTION SET TO NEXT ACTION SET
                [Qp, ydotbar] = iCAT_task(tasks{i}.A, tasks{i}.J, ...
                                           Qp, ydotbar, tasks{i}.xdotbar, ...
                                           1e-4, 0.01, 10);
            end
            %% 3. Last task: residual damping
            [~, ydotbar] = iCAT_task(eye(14), eye(14), Qp, ydotbar, zeros(14,1), 1e-4, 0.01, 10);

            % if bm_system.left_arm.grasped && bm_system.right_arm.grasped
            % 
            %     %disp(bm_system.left_arm.wJo);
            %     X_o1 = bm_system.left_arm.wJo*ydotbar(1:7);
            %     X_o2 = bm_system.right_arm.wJo*ydotbar(8:14);
            % 
            %     H_1 = bm_system.left_arm.wJo*pinv(bm_system.left_arm.wJo);
            %     H_2 = bm_system.right_arm.wJo*pinv(bm_system.right_arm.wJo);
            %     H_12 = [H_1 zeros(6,6);zeros(6,6) H_2];
            % 
            %     [v_ang, v_lin] = CartError(bm_system.left_arm.wTog , bm_system.left_arm.wTo);
            %     xdotbar = 1.0 * [v_ang; v_lin];
            %     xdotbar(1:3) = Saturate(xdotbar(1:3), 0.3);
            %     xdotbar(4:6) = Saturate(xdotbar(4:6), 0.3);
            % 
            %     mu_1 = 0 + norm(xdotbar-X_o1);
            %     mu_2 = 0 + norm(xdotbar-X_o2);
            % 
            %     xdot_t = (mu_1*X_o1+mu_2*X_o2)/(mu_1+mu_2);
            % 
            %     Xdot_t = [xdot_t;xdot_t];
            % 
            %     C = [H_1 -H_2];
            % 
            %     Xo_1_2 = H_12*(eye(12)-pinv(C)*C)*Xdot_t;
            % 
            %     % Cerca nelle celle l'oggetto x che ha x.taskID uguale a "LT"
            %     grasp_l_id = find(cellfun(@(x) x.id == "graspLeft", tasks), 1);
            %     grasp_r_id = find(cellfun(@(x) x.id == "graspRight", tasks), 1);
            % 
            %     tasks{grasp_l_id}.xdotbar = Xo_1_2(1:6,:);
            %     tasks{grasp_r_id}.xdotbar = Xo_1_2(7:12,:);
            % 
            %     tasks = [tasks(grasp_r_id), tasks(1:grasp_r_id-1), tasks(grasp_r_id+1:end)];
            %     tasks = [tasks(grasp_l_id), tasks(1:grasp_l_id-1), tasks(grasp_l_id+1:end)];
            % 
            % 
            %     for i = 1:length(tasks)
            %          %% TRANSITION FROM PREVIOUS ACTION SET TO NEXT ACTION SET
            %         [Qp, ydotbar] = iCAT_task(tasks{i}.A, tasks{i}.J, ...
            %                                    Qp, ydotbar, tasks{i}.xdotbar, ...
            %                                    1e-4, 0.01, 10);
            %     end
            %     %% 3. Last task: residual damping
            %     [~, ydotbar] = iCAT_task(eye(14), eye(14), Qp, ydotbar, zeros(14,1), 1e-4, 0.01, 10);
            % end
        end
    end
end