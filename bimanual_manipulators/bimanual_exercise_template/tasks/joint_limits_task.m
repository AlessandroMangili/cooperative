classdef joint_limits_task < Task   
    %Tool position control for a single arm
    properties
        diff_max;
        diff_min;
        threshold = 0.2;
        k = 0.5
        id;
    end

    methods
        function obj=joint_limits_task(robot_ID,taskID, id)
            obj.ID=robot_ID;
            obj.task_name=taskID;
            obj.id = id;
        end

        function updateReference(obj, robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;    
            end
            
            % obj.diff_max = robot.jlmax - robot.q;
            % obj.diff_min = robot.q - robot.jlmin;
            % if (obj.diff_max < obj.threshold)
            %     obj.xdotbar = 0.2 * obj.diff_max;
            % else
            %     obj.xdotbar = 0.2 * obj.diff_min;
            % end
            obj.xdotbar = 0.2 * robot.q;
            %limit the requested velocities...
            obj.xdotbar(1:7) = Saturate(obj.xdotbar(1:7), 0.3);
        end
        
        function updateJacobian(obj,robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;    
            end

            if obj.ID=='L'
                obj.J= [eye(7), zeros(7,7)];
            elseif obj.ID=='R'
                obj.J= [zeros(7,7), eye(7)];
            end
        end

        function updateActivation(obj, robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;    
            end

            obj.A = eye(7);

            threshold_activation = 0.2;

            for i = 1:7
                obj.A(i,i) = DecreasingBellShapedFunction(robot.jlmin(i), robot.jlmin(i)+threshold_activation, 0, 1, robot.q(i)) ...
                + IncreasingBellShapedFunction(robot.jlmax(i)-threshold_activation, robot.jlmax(i), 0, 1, robot.q(i));
            end
            % offset = 0.1;
            % if (obj.diff_max < obj.threshold)
            %     obj.A = obj.A * DecreasingBellShapedFunction(0, offset, 0, 1, obj.diff_max);
            % else
            %     obj.A = obj.A * IncreasingBellShapedFunction(0, offset, 0, 1, obj.diff_min);
            % end
        end
    end
end