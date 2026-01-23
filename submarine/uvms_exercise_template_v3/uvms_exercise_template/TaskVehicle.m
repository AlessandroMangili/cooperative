classdef TaskVehicle < Task   
   properties

    end


    methods
        function updateReference(obj, robot)
            [ang, lin] = CartError(robot.wTgv , robot.wTv);
            obj.xdotbar = - 0.2 * robot.vTw(1:3,1:3) * lin;
            % limit the requested velocities...
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.2);
            %obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.2);
        end
        function updateJacobian(obj, robot)
            Jt_a  = zeros(3,7); %J of arm
            Jt_v = [-eye(3) zeros(3, 3)]; %J of vehicle
            obj.J = [Jt_a Jt_v];
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(3);
        end
    end
end