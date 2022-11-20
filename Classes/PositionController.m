classdef PositionController < handle
 
    properties
        acceleration_des = zeros(3,1);
        omega_ref = zeros(3,1);
        omega_dot_ref = zeros(3,1);
        c_des = 0;
        D_POS;
        P_POS;
        g_vector;
        
        % Test Sliding Mode Control
        K1;
        K;
        GAMMA
    end
    
    methods
        function obj = PositionController()
            obj.g_vector = [0; 0; -9.81];
            obj.P_POS = diag([15, 15, 15]);
            obj.D_POS = diag([5, 5, 15]);
            
            % Test Sliding Mode Control
            obj.K1 = diag([1.85, 1.85, 1.85]);
            obj.K = diag([3, 3, 3]);
            obj.GAMMA = diag([7, 7, 7]);

        end
        
        function updateAcceleration(obj, generator, observer)
            obj.acceleration_des = obj.P_POS * (generator.r_ref - observer.r_adjust) + obj.D_POS * (generator.v_ref - observer.v_adjust) + generator.a_ref - obj.g_vector;
%             e = (generator.r_ref - observer.r_adjust);
%             e_dot = (generator.v_ref - observer.v_adjust);
%             sigma = obj.K1 * e + e_dot;
%             obj.acceleration_des = -obj.g_vector + generator.a_ref + obj.K1 * e_dot + obj.K * tanh(sigma) + obj.GAMMA * sigma;
        end
        
        function updateAngularRef(obj, generator ,observer)
            inv_c_des = 1.0 / observer.c_des;
            inv_cross = 1.0 / norm( cross(observer.ey_C, observer.ez_B_des) );
            term = 0.0;

            term = (observer.ey_B_des' *  generator.j_ref) * (-1.0);
            obj.omega_ref(1) = inv_c_des * term;

            term = observer.ex_B_des' *  generator.j_ref;
            obj.omega_ref(2) = inv_c_des * term;

            term = (observer.ex_C' *  observer.ex_B_des) * generator.phi_dot;
            term = term + (observer.ey_C' *  observer.ez_B_des) * obj.omega_ref(2);
            obj.omega_ref(3) = inv_cross * term;

            term = (observer.ey_B_des' * generator.s_ref) * (-1.0);
            term = term + (observer.ez_B_des' * generator.j_ref) * obj.omega_ref(1) * (-2.0);
            term = term + (observer.c_des * obj.omega_ref(2)) * obj.omega_ref(3);
            obj.omega_dot_ref(1) = inv_c_des * term;

            term = observer.ex_B_des' * generator.s_ref;
            term = term + (observer.ez_B_des' * generator.j_ref) * obj.omega_ref(2) * (-2.0);
            term = term - observer.c_des * obj.omega_ref(1) * obj.omega_ref(3);
            obj.omega_dot_ref(2) = inv_c_des * term;

            term = (observer.ey_C' * observer.ez_B_des) * obj.omega_dot_ref(2);
            term = term + (observer.ex_C' * observer.ex_B_des) * generator.phi_dot_dot;
            term = term + (observer.ex_C' * observer.ey_B_des) * (2.0) * generator.phi_dot * obj.omega_ref(3);
            term = term + (observer.ex_C' * observer.ez_B_des) * (-2.0) * generator.phi_dot * obj.omega_ref(2);
            term = term - (observer.ey_C' * observer.ey_B_des) * obj.omega_ref(1) * obj.omega_ref(2);
            term = term - (observer.ey_C' * observer.ez_B_des) * obj.omega_ref(1) * obj.omega_ref(3);
            obj.omega_dot_ref(3) = inv_cross * term;
        end
        
    end
end

