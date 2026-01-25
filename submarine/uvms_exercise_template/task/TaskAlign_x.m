classdef TaskAlign_x < Task   
   properties
        theta;
        id;
    end


    methods
        function obj = TaskAlign_x(id)
            obj.id = id;
            obj.theta = 0.0;
        end

        function updateReference(obj, robot)
            xRv = robot.wTv(1:3,1);
            [~, lin] = CartError(robot.wTg, robot.wTv);
            lin(3) = 0;

            ang =  skew(xRv)*lin;
            
            obj.theta = atan2(norm(ang), dot(xRv,lin));

            if(obj.theta < 0)
                obj.theta = 2* pi + obj.theta;
            end

            obj.xdotbar = -0.2 * (0.1 - obj.theta);
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        function updateJacobian(obj, robot)
            wRv = robot.wTv(1:3,1:3);
            [~, lin] = CartError(robot.wTg, robot.wTv);
            lin(3) = 0;
            ang =  skew(wRv(:,1)) * lin;
            ang = ang / norm(ang);

            obj.J = ang'*[zeros(3,7) zeros(3,3), wRv];
        end
        
        function updateActivation(obj, robot)
            obj.A = IncreasingBellShapedFunction(0.0, 0.1, 0, 1, obj.theta);
        end
    end
end