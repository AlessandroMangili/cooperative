classdef TaskVehicle < Task   
   properties
        id
    end


    methods
        function obj = TaskVehicle(id)
            obj.id = id;
        end

        function updateReference(obj, robot)
            [ang, lin] = CartError(robot.wTgv , robot.wTv);
            v_ref = -0.4 * lin;
            w_ref = -0.4 * ang;
        
            obj.xdotbar = [v_ref; w_ref];

            % limit the requested velocities...
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.4);
            obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.4);
        end
        function updateJacobian(obj, robot)
            Jt_a = zeros(6,7);           
            Jt_v = -eye(6);              
        
            obj.J = [Jt_a Jt_v];
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(6);
        end
    end
end