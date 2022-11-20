classdef Mixer < handle
 
    properties
        control_signal = zeros(4,1);
        thrust_torque_vector = zeros(4,1);
        B_MATRIX;
        f_hover;
        u_hover;
        inv_alpha;
        a;
        b;
        c;
    end
    
    methods
        function obj = Mixer()
%             obj.B_MATRIX = [0.0317    0.2109   -0.2109    1.5456;
%                             0.0317   -0.2109   -0.2109   -1.5456;
%                             0.0317   -0.2109    0.2109    1.5456;
%                             0.0317    0.2109    0.2109   -1.5456];
              obj.B_MATRIX = 1.0e+05 *[  0.8022,    5.3317,   -5.3317,    8.1575,
                                0.8022,   -5.3317,   -5.3317,   -8.1575,
                                0.8022,   -5.3317,    5.3317,    8.1575,
                                0.8022,    5.3317,    5.3317,   -8.1575,     ];      
              obj.inv_alpha = 1/(0.1571);
                         
           
              obj.u_hover = [0.1162; 0.1162; 0.1162; 0.1162];
              obj.f_hover = 0.3 * 9.81 * ones(4,1);
              obj.a = -7.349e-14;
              obj.b = 5.924e-07;
              obj.c = -0.1213;
        end
        
        function computeControlSignal(obj)
%             ControlSignal = obj.B_MATRIX * obj.thrust_torque_vector;
%             ControlSignal( ControlSignal > 1 ) = 1; ControlSignal( ControlSignal < 0 ) = 0;
%             obj.control_signal = sqrt(ControlSignal);
 
%               ControlSignal = (obj.B_MATRIX * obj.thrust_torque_vector - obj.f_hover) * obj.inv_alpha + obj.u_hover;
%               ControlSignal( ControlSignal > 1 ) = 1; ControlSignal( ControlSignal < 0 ) = 0;
%               obj.control_signal = ControlSignal;
            ControlSignal = obj.B_MATRIX * obj.thrust_torque_vector;
%             ControlSignal( ControlSignal > 9.3195 ) = 9.3195; ControlSignal( ControlSignal < 1.5696 ) = 1.5696;
            ControlSignal = obj.a * ControlSignal.^2 + obj.b * ControlSignal + obj.c;
            ControlSignal( ControlSignal > 1 ) = 1; ControlSignal( ControlSignal < 0 ) = 0;
            obj.control_signal = ControlSignal;
        end
        
        function setThrustTorqueSetpoint(obj, torque_vector, thrust)
            obj.thrust_torque_vector = [thrust; torque_vector];
        end
        
    end
end

