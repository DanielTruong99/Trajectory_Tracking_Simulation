classdef GeneralObserver < handle
 
    properties
        init_r = zeros(3,1);
        r_adjust = zeros(3,1);
        v_adjust = zeros(3,1);
        init_quat = zeros(1,4);
        quat_adjust = zeros(1,4);
        q_cmd_rp = zeros(1,4);
        q_error = zeros(1,4);
        q_error_rp = zeros(1,4);
        q_des = zeros(1,4);
        ex_B_des = zeros(3,1);
        ey_B_des = zeros(3,1);
        ez_B_des = zeros(3,1);
        ez_B = zeros(3,1);
        ex_C = zeros(3,1);
        ey_C = zeros(3,1);
        rotation_angle = 0.0;
        c_des = 0.0;
        alpha_mix;
        init_time; %usec
        current_time; %usec

        q_z_minus_alpha;
        q_x_pi;
    end
    
    methods
        function obj = GeneralObserver()
            obj.q_x_pi = [0, 1, 0, 0];
            obj.q_z_minus_alpha = [1, 0, 0, 0];
        end
        
        function computeQuadError(obj, acceleration_des, heading_des)

            obj.ez_B(1) = 2 * (obj.quat_adjust(2) * obj.quat_adjust(4) + obj.quat_adjust(1) * obj.quat_adjust(3));
            obj.ez_B(2) = 2 * (obj.quat_adjust(3) * obj.quat_adjust(4) - obj.quat_adjust(1) * obj.quat_adjust(2));
            obj.ez_B(3) = 2 * (obj.quat_adjust(1) * obj.quat_adjust(1) + obj.quat_adjust(4) * obj.quat_adjust(4)) - 1;


            obj.ez_B_des = acceleration_des;
            obj.ez_B_des = obj.ez_B_des / norm(obj.ez_B_des);

            obj.rotation_angle = acos( (obj.ez_B' * obj.ez_B_des) );

            if( abs(obj.rotation_angle) < 0.00017 )
            
                obj.q_error_rp = [1, 0, 0, 0];
            
            else
                rotation_axis_wf = cross(obj.ez_B, obj.ez_B_des);
                rotation_axis_wf = rotation_axis_wf / norm(rotation_axis_wf);
                rotation_axis_wf = rotation_axis_wf * sin( obj.rotation_angle/2 );
                
                obj.q_error_rp = [cos( obj.rotation_angle/2 ), rotation_axis_wf(1), rotation_axis_wf(2), rotation_axis_wf(3) ];
            end

            obj.ex_C = [cos(heading_des), sin(heading_des), 0]';
            obj.ey_C = [-sin(heading_des), cos(heading_des), 0]';

            if(obj.ez_B_des(3) < 0.0)
            
                obj.ex_B_des = - cross(obj.ey_C, obj.ez_B_des);
                obj.ex_B_des = obj.ex_B_des / norm(obj.ex_B_des);
            
            else
            
                obj.ex_B_des = cross(obj.ey_C, obj.ez_B_des);
                obj.ex_B_des = obj.ex_B_des / norm(obj.ex_B_des);
            end

            obj.ey_B_des = cross(obj.ez_B_des, obj.ex_B_des);
            obj.ey_B_des = obj.ey_B_des / norm(obj.ey_B_des);

            dcm_form_array = [obj.ex_B_des, obj.ey_B_des, obj.ez_B_des];
            obj.q_des(1) = sqrt(0.25 * abs(1 + trace(dcm_form_array)));
            obj.q_des(2) = sqrt( 0.25 * abs( 1 - trace(dcm_form_array) + 2 * dcm_form_array(1,1) ) );
            obj.q_des(3) = sqrt( 0.25 * abs( 1 - trace(dcm_form_array) + 2 * dcm_form_array(2,2) ) );
            obj.q_des(4) = sqrt( 0.25 * abs( 1 - trace(dcm_form_array) + 2 * dcm_form_array(3,3) ) );
            obj.q_des = obj.q_des ./ norm(obj.q_des);
            
            obj.q_cmd_rp = quatmultiply(obj.quat_adjust, obj.q_error_rp);
            q_mix = quatmultiply(quatconj(obj.q_cmd_rp), obj.q_des);
            obj.alpha_mix = 2 * atan2(q_mix(4), q_mix(1));

%             obj.c_des = acceleration_des' * obj.ez_B;
            obj.c_des = norm(acceleration_des);
        end
        
        function getInitMeasurement(obj, attitude_sub_s, local_position_sub_s)
            obj.init_quat = attitude_sub_s';
            obj.init_r = local_position_sub_s;


            euler_init = quat2eul(obj.init_quat);
            init_heading_angle = euler_init(1);
            obj.q_z_minus_alpha(1) = cos(init_heading_angle/2);
            obj.q_z_minus_alpha(4) = -sin(init_heading_angle/2);

            
        end
        
        
        function adjustMeasurement(obj, attitude_sub_s, local_position_sub_s, local_velocity_sub_s)
%             obj.quat_adjust =  quatmultiply(quatmultiply(quatconj( quatmultiply(obj.q_x_pi, obj.q_z_minus_alpha) ), attitude_sub_s'), obj.q_x_pi);
%             obj.r_adjust = (_q_x_pi * _q_z_minus_alpha).rotateVectorInverse(Vector3f(local_position_sub_s.x, local_position_sub_s.y, local_position_sub_s.z) - _init_r);
%             _v_adjust = (_q_x_pi * _q_z_minus_alpha).rotateVectorInverse(Vector3f(local_position_sub_s.vx, local_position_sub_s.vy, local_position_sub_s.vz)); 
            obj.quat_adjust = attitude_sub_s';
            obj.r_adjust = local_position_sub_s;
            obj.v_adjust = local_velocity_sub_s;
        end
        
    end
end

