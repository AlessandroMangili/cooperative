classdef joint_limits_task < Task   
    %Tool position control for a single arm
    properties
        threshold;
        lambda = 0.5
        id;
    end

    methods
        function obj=joint_limits_task(id, joint_threshold)
            obj.id = id;
            obj.threshold = joint_threshold;
        end

        function updateReference(obj, robot)            
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
        
            obj.xdotbar = Saturate(obj.xdotbar, obj.lambda);
        end
        
        function updateJacobian(obj,robot)
            obj.J= eye(7);
        end

        function updateActivation(obj, robot)
            obj.A = eye(7);
            threshold_activation = 0.2;

            for i = 1:7
                obj.A(i,i) = DecreasingBellShapedFunction(robot.jlmin(i), robot.jlmin(i)+threshold_activation, 0, 1, robot.q(i)) ...
                + IncreasingBellShapedFunction(robot.jlmax(i)-threshold_activation, robot.jlmax(i), 0, 1, robot.q(i));
            end
        end
    end
end