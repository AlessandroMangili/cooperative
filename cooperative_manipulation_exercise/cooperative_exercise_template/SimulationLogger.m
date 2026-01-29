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
        xdot_d       % desired object velocity
        xdot_nc      % non-coop cart vel
        xdot_a       % coop vel
        distance_t   % distance between tools
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

            obj.distance_t =  zeros(1, maxLoops);

            obj.xdot_d = zeros(6, maxLoops);
            obj.xdot_nc = zeros(6, maxLoops);
            obj.a = zeros(6, maxLoops);
            % l=zeros(1,obj.n);
            % for i=1:obj.n
            %     l(i)=length(action_set.actions{i});
            % end
            %obj.xdotbar_task=cell(length(action_set.actions), max(l), maxLoops);

            %maxDiagSize = max(cellfun(@(t) size(t.A,1), obj.tasks_set));
            obj.a = zeros(7, maxLoops, length(obj.tasks_set));
            
        end

        function update(obj, t, loop, xdot_d, xdot_nc, distance)
            % Store robot state
            obj.t(loop) = t;
            obj.q(:, loop) = obj.robot.q;
            obj.qdot(:, loop) = obj.robot.qdot;

            obj.distance_t(loop) = distance;

            obj.xdot_d(:, loop) = xdot_d;
            obj.xdot_nc(:, loop) = xdot_nc;
            obj.xdot_a(:, loop) = obj.robot.xdot_coop_vel;
            
             % modified plot
            for i = 1:length(obj.tasks_set)
                A = obj.tasks_set{i}.A;

                if isscalar(A)
                    diagA = repmat(A, 7, 1);
                else
                    diagA = diag(A);
            
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

                figure(4);
                plot(obj.t, obj.distance_t, 'LineWidth', 2);
                legend('tool ditance');

                figure('Name', 'Plot velocities');            
                titles = {'\omega_x', '\omega_y', '\omega_z', 'v_x', 'v_y', 'v_z'};

                for i = 1:6
                    if i <= 3
                        pos = (i-1)*2 + 1;
                    else
                        pos = (i-4)*2 + 2;
                    end
                    %subplot(3, 2, i);
                    ax = subplot(3,2,pos);
                    hold(ax, 'on');
                    %hold on;
                    h1 = plot(obj.t, obj.xdot_d(i, :), 'r', 'LineWidth', 2);

                    h2 = plot(obj.t, obj.xdot_nc(i, :), 'g', 'LineWidth', 2);

                    h3 = plot(obj.t, obj.xdot_a(i, :), 'b', 'LineWidth', 2, 'LineStyle', '--');

                    ylabel(titles{i});
                    grid on;

                    if i == 4
                        legend([h1, h2, h3], 'desired object vel', 'non copperative vel', 'cooperative vel');
                    end
                end

            end

    end
end