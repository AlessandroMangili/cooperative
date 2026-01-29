classdef joint_limits_task < Task   
    %Tool position control for a single arm
    properties
        threshold;
        lambda = 0.5
        id;
    end

    methods
        function obj=joint_limits_task(robot_ID,taskID, id, joint_threshold)
            obj.ID=robot_ID;
            obj.task_name=taskID;
            obj.id = id;
            obj.threshold = joint_threshold;
        end

        function updateReference(obj, robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;    
            end
            
             obj.xdotbar = zeros(7,1);
        
            for i = 1:7
                dist_min = robot.q(i) - robot.jlmin(i);
                dist_max = robot.jlmax(i) - robot.q(i);
        
                if dist_min < obj.threshold
                    obj.xdotbar(i) = obj.lambda * (obj.threshold - dist_min);
                elseif dist_max < obj.threshold
                    obj.xdotbar(i) = -obj.lambda * (obj.threshold - dist_max);
                end
            end
        
            obj.xdotbar = Saturate(obj.xdotbar, 0.3);
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

            %threshold_activation = 0.2;

            for i = 1:7
                obj.A(i,i) = DecreasingBellShapedFunction(robot.jlmin(i), robot.jlmin(i)+obj.threshold, 0, 1, robot.q(i)) ...
                + IncreasingBellShapedFunction(robot.jlmax(i)-obj.threshold, robot.jlmax(i), 0, 1, robot.q(i));
            end
        end
    end
end