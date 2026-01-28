classdef ActionManager < handle
    properties
        actions = {}      % cell array of actions (each action = stack of tasks)
        action_names = []     % names of actions
        unifying_actions
        unifying_actions_coop
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

         function addUnifyingTasks(obj, unified_set, unified_set_coop)
            obj.unifying_actions = unified_set;
            obj.unifying_actions_coop = unified_set_coop;
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

        function [ydotbar] = computeICAT(obj,bm_system, coop, dt)
            obj.actionSwitchTime = obj.actionSwitchTime + dt;

            if ~coop
                tasks = obj.unifying_actions;
            else
                tasks = obj.unifying_actions_coop;
            end
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

            if coop
                inCurrent(1) = 1;
                inPrev(1) = 1;
            end

            for i = 1:length(tasks)
                task = tasks{i};
                task.updateReference(bm_system);
                task.updateJacobian(bm_system);
                task.updateActivation(bm_system);

                if (tasks{i}.id == "GraspCoopLeft" || tasks{i}.id == "GraspCoopRight") && ~inCurrent(i)
                    alpha = 0;
                elseif (tasks{i}.id == "GraspCoopLeft" || tasks{i}.id == "GraspCoopRight") && inCurrent(i)
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
           
            %% 2. Perform ICAT (task-priority inverse kinematics) for the current Action
            nJ = length(bm_system.q); % number of joints
            ydotbar = zeros(nJ,1);
            Qp = eye(nJ);
            for i = 1:length(tasks)
                 %% TRANSITION FROM PREVIOUS ACTION SET TO NEXT ACTION SET
                [Qp, ydotbar] = iCAT_task(tasks{i}.A, tasks{i}.J, ...
                                           Qp, ydotbar, tasks{i}.xdotbar, ...
                                           1e-4, 0.01, 10);
            end
            %% 3. Last task: residual damping
            [~, ydotbar] = iCAT_task(eye(nJ), eye(nJ), Qp, ydotbar, zeros(nJ,1), 1e-4, 0.01, 10);
        end
    end
end