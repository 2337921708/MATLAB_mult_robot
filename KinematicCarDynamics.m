classdef KinematicCarDynamics < handle
    methods(Static)
        function state_next = forward_dynamics(state, control, timestep)
            % 普通状态向量
            x = state(1); y = state(2); theta = state(3);
            v = control(1); omega = control(2);
            state_next = [x + v*cos(theta)*timestep;
                   y + v*sin(theta)*timestep;
                   theta + omega*timestep]; % system r.h.s
        end
        
        function state_dot = dot_dynamics(state, control)
            % 普通状态向量
            x = state(1); y = state(2); theta = state(3);
            v = control(1); omega = control(2);
            state_dot = [  v*cos(theta);
                           v*sin(theta);
                           omega]; % system r.h.s
        end
        
        function f = forward_dynamics_opt(timestep)
            import casadi.*
            % 普通状态向量
            x = SX.sym('x'); y = SX.sym('y'); theta = SX.sym('theta');
            states = [x;y;theta]; 
            % 控制变量
            v = SX.sym('v'); omega = SX.sym('omega_L');
            controls_L = [v;omega];
            rhs = [x + v*cos(theta)*timestep;
                   y + v*sin(theta)*timestep;
                   theta + omega*timestep]; % system r.h.s
            f = Function('f',{states,controls_L},{rhs}); % nonlinear mapping function f(x,u)
        end
        
        % 跟随者局部误差动力学模型
        function f_fe = forward_follower_error_dynamics_opt(timestep)
            import casadi.*
            % 普通状态向量
            x = SX.sym('x'); y = SX.sym('y'); theta = SX.sym('theta');
            states = [x;y;theta]; 
            % 误差状态向量
            x_e = SX.sym('x_e'); y_e = SX.sym('y_e'); theta_e = SX.sym('theta_e');
            states_e = [x_e;y_e;theta_e]; 
            % 控制变量
            v = SX.sym('v'); omega = SX.sym('omega_L');
            controls_L = [v;omega];
            % 领航和跟随的向量
            x_d1 = SX.sym('x_d1'); y_d1 = SX.sym('x_d1');theta_d1= SX.sym('theta_d1');
            X_d1 = [x_d1;y_d1;theta_d1];
            % 参考轨迹变量(就是领航者的轨迹)
            x_r = SX.sym('x_r'); y_r = SX.sym('y_r'); theta_r = SX.sym('theta_r');
            v_r = SX.sym('v_r');omega_r = SX.sym('omega_r');
            refs = [x_r, y_r, theta_r,v_r,omega_r ];
            % 局部误差动力学模型
            rhs_fe = [omega*y_e+x_d1*omega_r*sin(theta_r-theta)+y_d1*omega_r*cos(theta_r-theta)+v_r*cos(theta_r-theta)-v;
                -omega*x_e-x_d1*omega_r*cos(theta_r-theta)+y_d1*omega_r*sin(theta_r-theta)+v_r*sin(theta_r-theta);
                omega_r - omega]; % system r.h.
            rhs_fe=states_e + rhs_fe.*timestep;
            f_fe = Function('f_fe',{states,states_e,controls_L,refs,X_d1},{rhs_fe}); % nonlinear mapping function f(x,u)
        end
        
        % 领导者局部误差动力学模型
        function f_e = forward_error_dynamics_opt(timestep)
            import casadi.*
            % 状态向量
            x_e = SX.sym('x_e'); y_e = SX.sym('y_e'); theta_e = SX.sym('theta_e');
            states_e = [x_e;y_e;theta_e];
            
            % 控制变量
            v = SX.sym('v'); omega = SX.sym('omega_L');
            controls_L = [v;omega];
            % 参考轨迹变量
            x_r = SX.sym('x_r'); y_r = SX.sym('y_r'); theta_r = SX.sym('theta_r');
            v_r = SX.sym('v_r');omega_r = SX.sym('omega_r');
            refs = [x_r, y_r, theta_r,v_r,omega_r ];
            
            % 局部误差动力学模型
            rhs_e = [y_e*omega-v+v_r*cos(theta_e);-x_e*omega+v_r*sin(theta_e);omega_r-omega]; % system r.h.s
            rhs_e=states_e + rhs_e.*timestep;
            f_e = Function('f_e',{states_e,controls_L,refs},{rhs_e}); % nonlinear mapping function f(x,u)
        end
        % 获得下一刻的状态量,用的是车身坐标系下的误差方程
        % ref_curr表示对应的参考轨迹向量
        % xk表示小车当前状态
        function error_state = get_Leader_initial_state(ref_curr, xk)
            sp_yaw = xk(3);
            
            T_lonlat2xy = [cos(sp_yaw), -sin(sp_yaw);
                sin(sp_yaw), cos(sp_yaw)];
            T_xy2lonlat = [cos(sp_yaw), sin(sp_yaw);
                -sin(sp_yaw), cos(sp_yaw)];
            error_xy = ref_curr(1:2) - xk(1:2); % 获得参考误差
            error_state = T_xy2lonlat * error_xy; % 得到横向误差和纵向误差
            % 计算横摆角误差
            error_yaw = ref_curr(3) - xk(3);
            while (-2*pi <= error_yaw && error_yaw <= 2*pi) == 0
                if (error_yaw >= 2*pi)
                    error_yaw = error_yaw - 2*pi;
                elseif (error_yaw <= -2*pi)
                    error_yaw = error_yaw + 2*pi;
                end
            end
            if (error_yaw > pi)
                error_yaw = error_yaw - 2*pi;
            elseif (error_yaw < -pi)
                error_yaw = error_yaw + 2*pi;
            end
            error_state(end+1) = error_yaw;
        end
        
        % 获得下一刻的状态量,用的是车身坐标系下的误差方程
        % ref_curr表示前车的状态
        % xk表示当前车的状态
        function error_state = get_Follower_initial_state(xL, xk, xd)
            theta = xk(3);
            delt = xL(3)-xk(3);
            
            T_lonlat2xy = [cos(delt), -sin(delt),  0;
                sin(delt), cos(delt) ,  0;
                0          , 0       ,  1];
            T_xy2lonlat = [cos(theta), sin(theta), 0;
                -sin(theta), cos(theta), 0;
                0           , 0        , 1];
            error_xy = xL - xk; % 获得参考误差
            error_state = T_xy2lonlat * error_xy-T_lonlat2xy*xd; % 得到横向误差和纵向误差
            % 计算横摆角误差
            error_yaw = error_state(3);
            while (-2*pi <= error_yaw && error_yaw <= 2*pi) == 0
                if (error_yaw >= 2*pi)
                    error_yaw = error_yaw - 2*pi;
                elseif (error_yaw <= -2*pi)
                    error_yaw = error_yaw + 2*pi;
                end
            end
            if (error_yaw > pi)
                error_yaw = error_yaw - 2*pi;
            elseif (error_yaw < -pi)
                error_yaw = error_yaw + 2*pi;
            end
            error_state(3) = error_yaw;
        end
    end
end