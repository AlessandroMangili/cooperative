classdef TaskManipulability < Task   
    properties
        id;
        arm_length;
        e;
    end

    methods
        function obj = TaskManipulability(id, arm_length)
            obj.id = id;
            obj.arm_length = arm_length * 0.6;
        end

        function updateReference(obj, robot)
            wTb = robot.wTv*robot.vTb;
            [~, lin] = CartError(robot.wTg , wTb);
            lin(3) = 0;
            obj.e = norm(lin) - obj.arm_length;
            obj.xdotbar = 0.2 * lin;
            % limit the requested velocities...
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.2);
        end

        function updateJacobian(obj, robot)
            Jt_a  = zeros(3,7);
            Jt_v = [blkdiag(eye(2), 0) zeros(3, 3)];
            obj.J = [Jt_a Jt_v];
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(3);
            obj.A = obj.A * IncreasingBellShapedFunction(0.0, 0.3, 0, 1, obj.e);
        end
    end
end