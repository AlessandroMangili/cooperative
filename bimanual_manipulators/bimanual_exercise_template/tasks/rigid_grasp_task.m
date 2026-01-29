classdef rigid_grasp_task < Task
    properties
        id;
    end

    methods
        function obj=rigid_grasp_task(id)
            obj.id = id;
        end
        
        function updateReference(obj, robot_system)
            obj.xdotbar = zeros(6,1);
        end
        
        function updateJacobian(obj, robot_system)        
            J_L = robot_system.left_arm.wJt;  
            J_R = robot_system.right_arm.wJt;              
            obj.J = [J_L, -J_R];
        end
        
        function updateActivation(obj, robot_system)
            obj.A = eye(6);
        end
    end
end
       