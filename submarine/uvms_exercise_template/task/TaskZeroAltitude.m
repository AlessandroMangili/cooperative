classdef TaskZeroAltitude < Task   
   properties
       id;
       threshold;
   end

    methods
        function obj = TaskZeroAltitude(id, threshold)
            obj.id = id;
            obj.threshold = threshold;
        end

        function updateReference(obj, robot)
            if (isempty(robot.altitude))
                alt = 2;
            else
                alt = robot.altitude;
            end
            obj.xdotbar = -0.2 * (alt - obj.threshold);
            obj.xdotbar = Saturate(obj.xdotbar, 0.1);
        end
        function updateJacobian(obj, robot)
            obj.J = [0 0 1] * [zeros(3,7) eye(3) zeros(3,3)];
        end
        
        function updateActivation(obj, robot)
            obj.A = 1;
        end
    end
end