classdef SimulationLogger < handle
    properties
        t            % time vector
        q            % joint positions
        qdot         % joint velocities
        a            % task activations (diagonal only)
        xdotbar_task % reference velocities for tasks (cell array)
        robot        % robot model
        action_set   % set of actions
        xdot         % End Effector velocity
        tasks_set    % set of tasks
        n            % lenght of tasks
    end

    methods
        function obj = SimulationLogger(maxLoops, robot, action_set)
            obj.robot = robot;
            obj.action_set = action_set;
            obj.tasks_set = action_set.unifying_actions_coop;

            obj.t = zeros(1, maxLoops);
            obj.q = zeros(7, maxLoops);
            obj.qdot = zeros(7, maxLoops);
            %Optional: plot the end-effector velocities
            obj.xdot=zeros(6,maxLoops);
            obj.n=length(obj.tasks_set);
            % l=zeros(1,obj.n);
            % for i=1:obj.n
            %     l(i)=length(action_set.actions{i});
            % end
            %obj.xdotbar_task=cell(length(action_set.actions), max(l), maxLoops);

            %maxDiagSize = max(cellfun(@(t) size(t.A,1), obj.tasks_set));
            obj.a = zeros(7, maxLoops, length(obj.tasks_set));
            
        end

        function update(obj, t, loop)
            % Store robot state
            obj.t(loop) = t;
            obj.q(:, loop) = obj.robot.q;
            obj.qdot(:, loop) = obj.robot.qdot;
            
             % modified plot
            for i = 1:length(obj.tasks_set)
                A = obj.tasks_set{i}.A;

                if isscalar(A)
                    % Task scalare → replica su 6 DOF
                    diagA = repmat(A, 7, 1);
                else
                    % Task vettoriale → usa la diagonale
                    diagA = diag(A);
            
                    % (opzionale) sicurezza: forzi comunque a 6
                    if length(diagA) < 7
                        diagA(end+1:7, 1) = diagA(end);
                    elseif length(diagA) > 7
                        diagA = diagA(1:7);
                    end
                end
            
                obj.a(:, loop, i) = diagA;
                obj.xdotbar_task{i, loop} = obj.tasks_set{i}.xdotbar;
            end

        end
        function plotAll(obj)
                % Example plotting for robot state
                figure;
                subplot(2,1,1);
                plot(obj.t, obj.q,'.-','LineWidth', 2);
                legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
                title('Joint positions');
                subplot(2,1,2);
                plot(obj.t, obj.qdot,'.-', 'LineWidth', 2);
                legend('qd_1','qd_2','qd_3','qd_4','qd_5','qd_6','qd_7');
                title('Joint velocities');
                %Optional: Plot the end effector velocities
                % figure;
                % plot(obj.t, obj.xdot,'.-', 'LineWidth', 2);
                % legend('wx','wy','wz','vx','vy','vz')
                % grid on
                % title('End-effector velocities');    
                colors = lines(size(obj.a,3));
                figure;
                for i = 1:size(obj.a,3)
                    subplot(size(obj.a,3),1,i);
                    plot(obj.t, squeeze(obj.a(:, :, i))', 'LineWidth', 1, 'Color', colors(i,:));
                    title(sprintf('Task: %s Activations (diagonal)', obj.tasks_set{i}.id));
                end

            end

    end
end