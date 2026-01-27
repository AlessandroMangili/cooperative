classdef minimum_altitude_task < Task   
    %Tool position control for a single arm
    properties
        desire_minimum_altitude;
        altitude;
        id;
    end

    methods
        function obj=minimum_altitude_task(robot_ID,taskID, desire_altitude, id)
            obj.ID=robot_ID;
            obj.task_name=taskID;
            obj.desire_minimum_altitude = desire_altitude;
            obj.id = id;
        end

        function updateReference(obj, robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;    
            end

            obj.altitude = robot.wTt(3,4);
            error = obj.desire_minimum_altitude - obj.altitude;
            obj.xdotbar = 0.5 * error;
            obj.xdotbar = Saturate(obj.xdotbar, 0.3);
        end
        
        function updateJacobian(obj,robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;    
            end
            tool_jacobian=robot.wJt;
            
            if obj.ID=='L'
                obj.J= [tool_jacobian(3,:), zeros(1,7)];
            elseif obj.ID=='R'
                obj.J= [zeros(1,7), tool_jacobian(3,:)];
            end
        end

        function updateActivation(obj, robot_system)
            th = obj.desire_minimum_altitude;
            offset = 0.05;
            obj.A = DecreasingBellShapedFunction(th, th+offset, 0, 1, obj.altitude);
        end
    end
end