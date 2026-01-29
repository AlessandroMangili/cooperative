classdef ActionManager < handle
    properties
        actions = {}             % cell array of actions (each action = stack of tasks)
        action_names = []        % names of actions
        unifying_actions         % unified list
        currentAction = 1        % index of currently active action
        previousAction = []      % set of the previous action
        actionSwitchTime = 0     % action switch
        transitionDuration = 1.0 % action transition
    end

    methods
        function addAction(obj, taskStack, action_name)
            obj.actions{end+1} = taskStack;
            obj.action_names{end+1} = action_name;
        end

         function addUnifyingTasks(obj, unified_set)
            obj.unifying_actions = unified_set;
         end

        function setCurrentAction(obj, action_name)
            disp('Actions saved:');
            disp(obj.action_names);
            disp(['Switching to current action: ', action_name]);

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

            in_current = ismember(string(task_ids), string(current_task_ids));
            in_previous    = ismember(string(task_ids), string(prev_task_ids));

            for i = 1:length(tasks)
                task = tasks{i};
                task.updateReference(bm_system);
                task.updateJacobian(bm_system);
                task.updateActivation(bm_system);

                if (tasks{i}.id == "rigidGrasp") && ~in_current(i)
                    alpha = 0;
                elseif (tasks{i}.id == "rigidGrasp") && in_current(i)
                    alpha = 1;
                elseif in_current(i) && ~in_previous(i)
                    % entering → fade in
                    alpha = IncreasingBellShapedFunction(0, obj.transitionDuration, 0, 1, obj.actionSwitchTime);
                elseif ~in_current(i) && in_previous(i)
                    % leaving → fade out
                    alpha = DecreasingBellShapedFunction(0, obj.transitionDuration, 0, 1, obj.actionSwitchTime);
                elseif ~in_current(i) && ~in_previous(i)
                    % steady → normal activation
                    alpha = 0;
                elseif in_current(i) && in_previous(i)
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
        end
    end
end