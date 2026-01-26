classdef ActionManager < handle
    properties
        actions = {}      % cell array of actions (each action = stack of tasks)
        action_names = []     % names of actions
        unifying_actions = {}
        currentAction = 1 % index of currently active action
        previousAction = []
        actionSwitchTime = 0
        transitionDuration = 2.0
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
            disp('=== DEBUG setCurrentAction ===');
            disp(['Cercando azione: ', action_name]);
            disp(['Numero di azioni registrate: ', num2str(length(obj.action_names))]);
            disp('Nomi delle azioni registrate:');
            disp(obj.action_names);
            disp('==============================');

            idx = find([obj.action_names{:}] == action_name, 1);
            if isempty(idx)
                error('Action "%s" not found.', action_name);
            end

            if idx ~= obj.currentAction
                obj.previousAction = obj.currentAction;
                obj.currentAction  = idx;
                obj.actionSwitchTime = tic;
            end
        end

        function [v_nu, qdot] = computeICAT(obj, robot)
            tasks = obj.unifying_actions;
            task_ids = [];
            for i = 1:length(tasks)
                task_ids{i} = tasks{i}.id;
            end

            % compute blending ratio
            if obj.actionSwitchTime ~= 0
                t = toc(obj.actionSwitchTime);
                alpha = min(t / obj.transitionDuration, 1);
            else
                alpha = 1;
            end
            %check discontinuità -> sostituisci con brll shape

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

           %% DA CAMBIARE CON BELL SHAPE FUNCITONS

            for i = 1:length(tasks)
                task = tasks{i};
                task.updateReference(robot);
                task.updateJacobian(robot);
                task.updateActivation(robot);

                if inCurrent(i) && ~inPrev(i)
                    % entering → fade in
                    task.A = task.A * alpha;
                elseif ~inCurrent(i) && inPrev(i)
                    % leaving → fade out
                    task.A = task.A * (1 - alpha);
                elseif ~inCurrent(i) && ~inPrev(i)
                    % steady → normal activation
                    task.A = task.A * 0;
                end
            end
            

            % % Get current action
            % tasks = obj.actions{obj.currentAction};
            % 
            % % 1. Update references, Jacobians, activations
            % for i = 1:length(tasks)
            %     tasks{i}.updateReference(robot);
            %     tasks{i}.updateJacobian(robot);
            %     tasks{i}.updateActivation(robot);
            % end

            % 2. Perform ICAT (task-priority inverse kinematics)
            ydotbar = zeros(13,1);
            Qp = eye(13);
            for i = 1:length(tasks)
                [Qp, ydotbar] = iCAT_task(tasks{i}.A, tasks{i}.J, ...
                                           Qp, ydotbar, tasks{i}.xdotbar, ...
                                           1e-4, 0.01, 10);
            end

            % 3. Last task: residual damping
            [~, ydotbar] = iCAT_task(eye(13), eye(13), Qp, ydotbar, zeros(13,1), 1e-4, 0.01, 10);

            % 4. Split velocities for vehicle and arm
            qdot = ydotbar(1:7);
            v_nu = ydotbar(8:13); % projected on the vehicle frame
        end
    end
end