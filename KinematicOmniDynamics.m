% 全向轮的一些运动学方程
classdef KinematicOmniDynamics < KinematicCarDynamics
    methods(Static)
        function state_next = forward_dynamics(state, control, timestep)
            % 普通状态向量
            x = state(1); y = state(2);
            v_x = control(1); v_y = control(2);
            state=[x;y];
            state_dot=[v_x;
                       v_y];       
            state_next = state+state_dot*timestep;
        end
        
        function state_dot = dot_dynamics(control)
            % 普通状态向量
            v_x = control(1); v_y = control(2);
            state_dot=[v_x;
                       v_y];
        end
        
        function f = forward_dynamics_opt(timestep)
            import casadi.*
            % 普通状态向量
            x = SX.sym('x'); y = SX.sym('y'); 
            state = [x;y]; 
            % 控制变量
            v_x = SX.sym('v_x'); v_y = SX.sym('v_y');
            controls_L = [v_x;v_y];
            state_dot=[v_x;
                       v_y]; 
            rhs = state+state_dot*timestep;
            f = Function('f',{state,controls_L},{rhs}); % nonlinear mapping function f(x,u)
        end
        
        % 跟随者局部误差动力学模型
        function f_fe = forward_follower_error_dynamics_opt(timestep)
            import casadi.*
            % 误差状态向量
            x_e = SX.sym('x_e'); y_e = SX.sym('y_e'); 
            states_e = [x_e;y_e]; 
            % 控制变量
            v_x = SX.sym('v_x'); v_y = SX.sym('v_y');
            controls_L = [v_x;v_y];

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
        
        % 全向轮的领航者和跟随者的预测模型是几乎相同的，可以共用一个模型
        function f_e = forward_error_dynamics_opt(timestep)
            import casadi.*
            % 状态向量
            x_e = SX.sym('x_e'); y_e = SX.sym('y_e');
            states_e = [x_e;y_e];
            
            % 控制变量
            v_x = SX.sym('v_x'); v_y = SX.sym('v_y');
            controls_L = [v_x;v_y];
            % 参考轨迹变量
            x_r = SX.sym('x_r'); y_r = SX.sym('y_r');
            v_xr = SX.sym('v_xr');v_yr = SX.sym('v_yr');
            refs = [x_r, y_r, v_xr, v_yr];
            
            % 局部误差动力学模型
            rhs_e = [v_xr-v_x; v_yr-v_y]; % system r.h.s
            rhs_e=states_e + rhs_e.*timestep;
            f_e = Function('f_e',{states_e,controls_L,refs},{rhs_e}); % nonlinear mapping function f(x,u)
        end

        % 表示全向轮的领航者的误差获取
        function error_xy = get_Leader_initial_state(ref_curr, xk)
            error_xy = ref_curr(1:2) - xk; % 获得参考误差         
        end
        
        % 表示全向轮的跟随者的误差获取
        function error_xy = get_Follower_initial_state(xL, xk, xd)
            error_xy = xL(1:2) - xk-xd; % 获得参考误差   
        end
    end
end