classdef coop_movement_task < Task
    properties
        id;
    end

    methods
        function obj = coop_movement_task(id)
            obj.id = id;
            obj.A = zeros(6);  % per plot
        end

        function updateReference(obj, robot)
            obj.xdotbar = robot.xdot_coop_vel;
        end

        function updateJacobian(obj, robot)
            obj.J = robot.wJo;
        end

        function updateActivation(obj, robot)
            obj.A = eye(6);
        end
    end
end