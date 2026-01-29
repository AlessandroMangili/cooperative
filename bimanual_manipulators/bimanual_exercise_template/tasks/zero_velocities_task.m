classdef zero_velocities_task < Task   
    properties
        id;
    end

    methods
        function obj=zero_velocities_task(robot_ID,taskID, id)
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
            obj.xdotbar = zeros(7,1);
        end
        
        function updateJacobian(obj,robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;    
            end
            if obj.ID=='L'
                obj.J=[eye(7), zeros(7, 7)];
            elseif obj.ID=='R'
                obj.J=[zeros(7, 7), eye(7)];
            end
        end

        function updateActivation(obj, robot_system)
            obj.A = eye(7);
        end
    end
end