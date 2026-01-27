classdef zero_velocities_task < Task   
    %Tool position control for a single arm
    properties
        id;
    end

    methods
        function obj=zero_velocities_task(id)
            obj.id = id;
        end
        function updateReference(obj, robot)
            obj.xdotbar = zeros(7,1);
        end
        
        function updateJacobian(obj,robot)
           obj.J=eye(7);
        end

        function updateActivation(obj, robot)
            obj.A = eye(7);
        end
    end
end