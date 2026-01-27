classdef coop_rigid_grasp_task < Task
    properties
        id;
    end

    methods
        function obj = coop_rigid_grasp_task(id)
            obj.id = id;
        end

        function updateReference(obj, robot)
            obj.xdotbar = -robot.xdot_coop_vel;
        end

        function updateJacobian(obj, robot)
            obj.J = robot.wJo;
        end

        function updateActivation(obj, robot)
            obj.A = eye(6);
        end
    end
end