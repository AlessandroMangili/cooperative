classdef TaskFixedBase < Task
   properties
        id;
        lin_desired;
        first;
    end


    methods
        function obj = TaskFixedBase(id)
            obj.id = id;
            obj.first = true;
            obj.lin_desired = zeros(3,1);
        end

        function updateReference(obj, robot)
            [~, lin] = CartError(robot.wTgv , robot.wTv);

            if obj.first && ~isempty(robot.altitude) && robot.altitude < 0.02 && robot.altitude > 0.01 %& manip.
                obj.lin_desired = lin;
                obj.first = false;
            end
            lin_err = lin - obj.lin_desired;
            v_ref = -0.2 * robot.vTw(1:3,1:3) * lin_err;

            obj.xdotbar = v_ref;
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.2);
        end
        function updateJacobian(obj, robot)
            Jt_a = zeros(3,7);           % braccio non contribuisce
            Jt_v = [blkdiag(eye(2), 0) zeros(3, 3)];  %J of vehicle
        
            obj.J = [Jt_a Jt_v];
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(3);
        end
    end
end