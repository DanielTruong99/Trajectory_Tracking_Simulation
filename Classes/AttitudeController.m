classdef AttitudeController < handle
 
    properties
        omega_des;
        q_error;

        
        P_RP;
        P_Y;
        P;
        TAU;
        OMEGA_Z_LIMIT;
    end
    
    methods
        function obj = AttitudeController()
            obj.TAU = 0.09;
            obj.P_RP = 2.0 / obj.TAU;
            obj.P_Y = 10.0;
            obj.P = 0.45;
            obj.OMEGA_Z_LIMIT = (2 * sin(0.5 * 3.14 * obj.P)) / obj.TAU;
        end
        
        function updateAngularVelocityDesired(obj, observer, position_controller)
            
            
            obj.q_error = quatmultiply( quatmultiply( quatconj(observer.quat_adjust), observer.q_cmd_rp ), [cos(obj.P * observer.alpha_mix/2), 0, 0, sin(obj.P * observer.alpha_mix/2)]);

            if( obj.q_error(1) < 0 )
            
                obj.omega_des = - obj.P_RP * obj.q_error(2:end)';
            
            else
            
                obj.omega_des = obj.P_RP * obj.q_error(2:end)';
            end

            if(observer.ex_B_des(1) == 0 && observer.ex_B_des(2) == 0 && observer.ex_B_des(3) == 0)
            
                obj.omega_des(3) = 0.0;
            end

            obj.omega_des = obj.omega_des + position_controller.omega_ref;

            if( obj.omega_des(3) > obj.OMEGA_Z_LIMIT )
            
                obj.omega_des(3) = obj.OMEGA_Z_LIMIT;
            
            elseif( obj.omega_des(3) < - obj.OMEGA_Z_LIMIT )
            
                obj.omega_des(3) = -obj.OMEGA_Z_LIMIT;
            end

        end
        
        
    end
end

