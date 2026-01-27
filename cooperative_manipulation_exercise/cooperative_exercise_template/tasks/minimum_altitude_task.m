classdef minimum_altitude_task < Task   
    %Tool position control for a single arm
    properties
        desire_minimum_altitude;
        altitude;
        id;
    end

    methods
        function obj=minimum_altitude_task(id, desire_altitude)
            obj.desire_minimum_altitude = desire_altitude;
            obj.id = id;
        end

        function updateReference(obj, robot)
            obj.altitude = robot.wTt(3,4);
            error = obj.desire_minimum_altitude - obj.altitude;
            obj.xdotbar = 0.5 * error;
            % limit the requested velocities...
            obj.xdotbar = Saturate(obj.xdotbar, 0.5);
        end
        
        function updateJacobian(obj,robot)
            obj.J= robot.wJt(3,:);
        end

        function updateActivation(obj, robot)
            th = obj.desire_minimum_altitude;
            offset = 0.05;
            obj.A = DecreasingBellShapedFunction(th, th+offset, 0, 1, obj.altitude);
        end
    end
end