classdef TaskManipulability < Task   
    properties
        id;
        arm_length;
        lin;
    end

    methods
        function obj = TaskManipulability(id, arm_length)
            obj.id = id;
            obj.arm_length = arm_length;
        end

        function updateReference(obj, robot)
            wTb = robot.wTv * robot.vTb;
            [~, obj.lin] = CartError(robot.wTg , wTb);
            obj.lin(3) = 0;
            
            if norm(obj.lin) > 1e-6
                dir_w = obj.lin / norm(obj.lin);
            else
                dir_w = zeros(3,1);
            end
        
            wRv = robot.wTv(1:3,1:3);
            dir_v = wRv' * dir_w;
        
            obj.xdotbar = 0.4 * [dir_v(1); dir_v(2); 0];

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
            obj.A = obj.A * IncreasingBellShapedFunction(obj.arm_length * 0.6, obj.arm_length * 0.8, 0, 1, norm(obj.lin));
        end
    end
end