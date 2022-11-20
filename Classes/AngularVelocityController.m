classdef AngularVelocityController < handle
 
    properties
        omega_des;
        omega_dot_des;
        torque_des;
        J;
        P_ATT;
    end
    
    methods
        function obj = AngularVelocityController()
            obj.P_ATT = diag([18, 17, 15]);
            obj.J = diag([0.0144, 0.0145, 0.0156]);
        end
        
        function updateTorqueDesired(obj, angular_velocity_mes)
            obj.torque_des = obj.J * obj.P_ATT * (obj.omega_des - angular_velocity_mes) + obj.J * obj.omega_dot_des + cross(angular_velocity_mes, obj.J * angular_velocity_mes);
        end
        
        function updateAngularVelocityDesired(obj, angular_velocity_des, angular_velocity_dot_des)
            obj.omega_des = angular_velocity_des;
            obj.omega_dot_des = angular_velocity_dot_des;

        end
        
    end
end

