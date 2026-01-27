classdef rigid_move_task < Task   
    properties
        id;
    end

    methods
        function obj=rigid_move_task(id)
            obj.id = id;
        end
        function updateReference(obj, robot)
         [v_ang, v_lin] = CartError(robot.wTog , robot.wTo);
         if robot.grasped
            robot.dist_to_goal=v_lin;
            robot.rot_to_goal=v_ang;
         end
         obj.xdotbar = 1.0 * [v_ang; v_lin];

         obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.3);
         obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.3);
        end
        
        function updateJacobian(obj,robot)           
            obj.J=robot.wJo;
        end

        function updateActivation(obj, robot)
            obj.A = eye(6);
        end
    end
end