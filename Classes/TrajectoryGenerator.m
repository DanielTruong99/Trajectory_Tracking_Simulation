classdef TrajectoryGenerator < handle
 
    properties
        r_ref = zeros(3,1);
        v_ref = zeros(3,1);
        a_ref = zeros(3,1);
        j_ref = zeros(3,1);
        s_ref = zeros(3,1);
        phi = 0;
        phi_dot = 0;
        phi_dot_dot = 0;    
    end
    
    methods
        function obj = TrajectoryGenerator()
            
        end
        
        function updateTrajectorieStates(obj, observer)
            delta_t = (observer.current_time - observer.init_time); % seconds
            obj.r(delta_t);
            obj.v(delta_t);
            obj.a(delta_t);
            obj.j(delta_t);
            obj.s(delta_t);
            obj.p(delta_t);
            obj.p_d(delta_t);
            obj.p_d_d(delta_t);
        end
        
        function r(obj, delta_t)
            xc = 0;
            yc = 1.5;
            R = 1.5;
            v = 1.5;
            w0 = v/R;
            a = 2;
            p0 = -pi/2;
            acc = 0.12;
            
            if(delta_t < 5)
                obj.r_ref(1) = 0;
                obj.r_ref(2) = 0;
                obj.r_ref(3) = 1.5;
            elseif(delta_t < 9)
                obj.r_ref(1) = 0;
                obj.r_ref(2) = 0;
                obj.r_ref(3) = 1.5;
            else
                t = delta_t - 9;
                obj.r_ref(1) = xc + R*cos(p0 + t*w0 + (w0*(exp(-a*t) - 1))/a);
                obj.r_ref(2) = yc + R*sin(p0 + t*w0 + (w0*(exp(-a*t) - 1))/a);
                obj.r_ref(3) = 1.5;
            end
            
        end
        
        
        function v(obj, delta_t)
            xc = 0;
            yc = 1.5;
            R = 1.5;
            v = 1.5;
            w0 = v/R;
            a = 2;
            p0 = -pi/2;
            acc = 0.12;
            
            if(delta_t < 5)
                obj.v_ref(1) = 0;
                obj.v_ref(2) = 0;
                obj.v_ref(3) = 0;
            elseif(delta_t < 9)
                obj.v_ref(1) = 0;
                obj.v_ref(2) = 0;
                obj.v_ref(3) = 0;
            else
                t = delta_t - 9;                
                obj.v_ref(1) = -R*sin(p0 + t*w0 + (w0*(exp(-a*t) - 1))/a)*(w0 - w0*exp(-a*t));
                obj.v_ref(2) = R*cos(p0 + t*w0 + (w0*(exp(-a*t) - 1))/a)*(w0 - w0*exp(-a*t));
                obj.v_ref(3) = 0;
            end
        end  
        
        function a(obj, delta_t)
            xc = 0;
            yc = 1.5;
            R = 1.5;
            v = 1.5;
            w0 = v/R;
            a = 2;
            p0 = -pi/2;
            acc = 0.12;
            
            if(delta_t < 5)
                obj.a_ref(1) = 0;
                obj.a_ref(2) = 0;
                obj.a_ref(3) = 0;
            elseif(delta_t < 9)
                obj.a_ref(1) = 0;
                obj.a_ref(2) = 0;
                obj.a_ref(3) = 0;
            else
                t = delta_t - 9;
                obj.a_ref(1) = - R*cos(p0 + t*w0 + (w0*(exp(-a*t) - 1))/a)*(w0 - w0*exp(-a*t))^2 - R*a*w0*sin(p0 + t*w0 + (w0*(exp(-a*t) - 1))/a)*exp(-a*t);
                obj.a_ref(2) = R*a*w0*cos(p0 + t*w0 + (w0*(exp(-a*t) - 1))/a)*exp(-a*t) - R*sin(p0 + t*w0 + (w0*(exp(-a*t) - 1))/a)*(w0 - w0*exp(-a*t))^2;
                obj.a_ref(3) = 0;            
            end
        end 
        
        function j(obj, delta_t)
            xc = 0;
            yc = 1.5;
            R = 1.5;
            v = 1.5;
            w0 = v/R;
            a = 2;
            p0 = -pi/2;
            
            if(delta_t < 5)
                obj.j_ref(1) = 0;
                obj.j_ref(2) = 0;
                obj.j_ref(3) = 0;
            elseif(delta_t < 9)
                obj.j_ref(1) = 0;
                obj.j_ref(2) = 0;
                obj.j_ref(3) = 0;
            else
                t = delta_t - 9;
                obj.j_ref(1) = R*sin(p0 + t*w0 + (w0*(exp(-a*t) - 1))/a)*(w0 - w0*exp(-a*t))^3 + R*a^2*w0*sin(p0 + t*w0 + (w0*(exp(-a*t) - 1))/a)*exp(-a*t) - 3*R*a*w0*cos(p0 + t*w0 + (w0*(exp(-a*t) - 1))/a)*exp(-a*t)*(w0 - w0*exp(-a*t));
                obj.j_ref(2) = - R*cos(p0 + t*w0 + (w0*(exp(-a*t) - 1))/a)*(w0 - w0*exp(-a*t))^3 - R*a^2*w0*cos(p0 + t*w0 + (w0*(exp(-a*t) - 1))/a)*exp(-a*t) - 3*R*a*w0*sin(p0 + t*w0 + (w0*(exp(-a*t) - 1))/a)*exp(-a*t)*(w0 - w0*exp(-a*t));
                obj.j_ref(3) = 0;
            end

        end     
        
        function s(obj, delta_t)
            xc = 0;
            yc = 1.5;
            R = 1.5;
            v = 1.5;
            w0 = v/R;
            a = 2;
            p0 = -pi/2;
            
            if(delta_t < 5)
                obj.s_ref(1) = 0;
                obj.s_ref(2) = 0;
                obj.s_ref(3) = 0;
            elseif(delta_t < 9)
                obj.s_ref(1) = 0;
                obj.s_ref(2) = 0;
                obj.s_ref(3) = 0;
            else
                t = delta_t - 9;
                obj.s_ref(1) = R*cos(p0 + t*w0 + (w0*(exp(-a*t) - 1))/a)*(w0 - w0*exp(-a*t))^4 - R*a^3*w0*sin(p0 + t*w0 + (w0*(exp(-a*t) - 1))/a)*exp(-a*t) - 3*R*a^2*w0^2*cos(p0 + t*w0 + (w0*(exp(-a*t) - 1))/a)*exp(-2*a*t) + 4*R*a^2*w0*cos(p0 + t*w0 + (w0*(exp(-a*t) - 1))/a)*exp(-a*t)*(w0 - w0*exp(-a*t)) + 6*R*a*w0*sin(p0 + t*w0 + (w0*(exp(-a*t) - 1))/a)*exp(-a*t)*(w0 - w0*exp(-a*t))^2;
                obj.s_ref(2) =   R*sin(p0 + t*w0 + (w0*(exp(-a*t) - 1))/a)*(w0 - w0*exp(-a*t))^4 + R*a^3*w0*cos(p0 + t*w0 + (w0*(exp(-a*t) - 1))/a)*exp(-a*t) - 3*R*a^2*w0^2*sin(p0 + t*w0 + (w0*(exp(-a*t) - 1))/a)*exp(-2*a*t) - 6*R*a*w0*cos(p0 + t*w0 + (w0*(exp(-a*t) - 1))/a)*exp(-a*t)*(w0 - w0*exp(-a*t))^2 + 4*R*a^2*w0*sin(p0 + t*w0 + (w0*(exp(-a*t) - 1))/a)*exp(-a*t)*(w0 - w0*exp(-a*t));
                obj.s_ref(3) = 0;
            end
        end     
        
        function p(obj, delta_t)

            if delta_t < 5
                obj.phi = 0;
            elseif delta_t < 9
                obj.phi = 0;
            else
                obj.phi = 0;
            end
        end             

        function p_d(obj, delta_t)

            if delta_t < 5
                obj.phi_dot = 0;
            elseif delta_t < 9
                obj.phi_dot = 0;
            else
                obj.phi_dot = 0;
            end
        end   
        
        function p_d_d(obj, delta_t)

            if delta_t < 5
                obj.phi_dot_dot = 0;
            elseif delta_t < 9
                obj.phi_dot_dot = 0;
            else
                obj.phi_dot_dot = 0;
            end
        end            
        
        
    end
end

