classdef TaskAltitude < Task   
   properties
        threshold_altitude = 2.0;
        id = "altitude"
    end


    methods
        function updateReference(obj, robot)
            %alt = 0;
            if(isempty(robot.altitude))
                alt = 2;
            else
                alt = robot.altitude;
            end
            error = obj.threshold_altitude - alt;
            obj.xdotbar = 0.5 * error;
            obj.xdotbar = Saturate(obj.xdotbar, 0.5);
        end
        function updateJacobian(obj, robot)
            obj.J = [0 0 1] * [zeros(3,7) eye(3) zeros(3,3)];
        end
        
        function updateActivation(obj, robot)
            %alt = 0;
            if(isempty(robot.altitude))
                alt = 2;
            else
                alt = robot.altitude;
            end
            obj.A = DecreasingBellShapedFunction(2.0, 3.0, 0,1, alt);
        end
    end
end