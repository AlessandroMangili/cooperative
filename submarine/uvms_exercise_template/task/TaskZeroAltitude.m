classdef TaskZeroAltitude < Task   
   properties
       id = "zero_altitude"
    end


    methods
        function updateReference(obj, robot)
            alt = 0;
            if (isempty(robot.altitude))
                alt = 2;
            else
                alt = robot.altitude;
            end
            obj.xdotbar = -0.2 * alt;
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        function updateJacobian(obj, robot)
            obj.J = [0 0 1] * [zeros(3,7) eye(3) zeros(3,3)];
        end
        
        function updateActivation(obj, robot)
            obj.A = 1;
        end
    end
end