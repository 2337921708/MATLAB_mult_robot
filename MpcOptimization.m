classdef MpcOptimization < handle
    properties
        opti
        variables_xe
        variables_u
        costs
        dynamics_opt
        error_dynamics_opt
        state
        x
        obs_distance
    end
    
    methods
        function self = MpcOptimization(dynamics_opt, error_dynamics_opt) % 类的初始化函数
            import casadi.*
            self.opti = NaN;
            self.variables_xe = [];
            self.variables_u = [];
            self.costs = 0;
            self.dynamics_opt = dynamics_opt;
            self.error_dynamics_opt=error_dynamics_opt;
            self.state = [];
            self.x = {};
            self.obs_distance=[];
        end
        
        function set_state(self, state)
            self.state = state;
        end
        
        function initialize_variables(self, param)
            self.variables_xe = self.opti.variable(2, param.horizon + 1);
            self.variables_u = self.opti.variable(2, param.horizon);
        end
        function add_initial_condition_constraint(self)
            self.opti.subject_to(self.variables_xe(:,1) == self.state.xe);
            self.x{end+1} = self.state.x;
        end
        
        function add_input_constraint(self, param)
            for i =1:param.horizon
                
                % input constraints
                self.opti.subject_to(self.variables_u(1,i) <= param.vx_max)
                self.opti.subject_to(param.vx_min <= self.variables_u(1,i))
                self.opti.subject_to(self.variables_u(2, i) <= param.vy_max)
                self.opti.subject_to(param.vy_min <= self.variables_u(2, i))
            end
        end
        
        function add_input_derivative_constraint(self, param)
            
            for i = 1:param.horizon - 1
                self.opti.subject_to(self.variables_u(1,i+1)-self.variables_u(1,i) <= param.v_dot_max)
                self.opti.subject_to(self.variables_u(1,i+1)-self.variables_u(1,i) >= param.v_dot_min)
                self.opti.subject_to(self.variables_u(2,i+1)-self.variables_u(2,i) <= param.omegadot_max)
                self.opti.subject_to(self.variables_u(2,i+1)-self.variables_u(2,i) >= param.omegadot_min)
            end
            self.opti.subject_to(self.variables_u(1,1) - self.state.u(1) <= param.v_dot_max)
            self.opti.subject_to(self.variables_u(1,1) - self.state.u(1) >= param.v_dot_min)
            self.opti.subject_to(self.variables_u(2,1) - self.state.u(2) <= param.omegadot_max)
            self.opti.subject_to(self.variables_u(2,1) - self.state.u(2) >= param.omegadot_min)
            
        end
        
        function get_predict_states(self,param)
            for i =1:param.horizon
                self.x{end+1} = self.dynamics_opt(self.x{i}, self.variables_u(:, i));
            end
        end
        
        function add_follower_error_dynamics_constraint(self, param, reference_trajectory,xd)
            for i =1: param.horizon
                self.opti.subject_to(self.variables_xe(:, i + 1) == ...
                    self.error_dynamics_opt(self.x{i},self.variables_xe(:, i), ...
                    self.variables_u(:, i),reference_trajectory(:, i),xd));
            end
        end
        
        function add_leader_error_dynamics_constraint(self, param, reference_trajectory)
            for i =1: param.horizon
                self.opti.subject_to(self.variables_xe(:, i + 1) == ...
                    self.error_dynamics_opt(...
                    self.variables_xe(:, i), self.variables_u(:, i), reference_trajectory(:, i))...
                    );
            end
        end
        
        
        
        function add_reference_trajectory_tracking_cost(self, param)
            import casadi.*
            for i =1: param.horizon-1
                self.costs = self.costs + mtimes(self.variables_xe(:,i)',...
                    mtimes(param.mat_Q, self.variables_xe(:,i)));
            end
            self.costs = self.costs +mtimes(...
                self.variables_xe(:,end)', mtimes(param.terminal_Q, self.variables_xe(:,end))...
                );
            
        end
%       在全向轮下领导者和跟随者用同样一个控制权重方程
        function add_input_stage_cost_leader(self, param, reference_trajectory)
            for i = 1:param.horizon
                con = self.variables_u(:,i); 
                temp_v_xL = con(1);
                temp_v_yL = con(2);
                temp_v_xr = reference_trajectory(3,i);
                temp_v_yr = reference_trajectory(4,i);
                
                u_Le = [temp_v_xL-temp_v_xr;temp_v_yL-temp_v_yr]; % 获得控制变量的误差
         
                self.costs = self.costs + mtimes(u_Le',...
                    mtimes(param.mat_R, u_Le));
            end
        end
%       为跟随者输入和参考输入的代价函数        
        function add_input_stage_cost_follower(self, param, reference_trajectory, xd)
            for i = 1:param.horizon
                st = self.variables_xe(:,i);  
                con = self.variables_u(:,i); 
                temp_theta_L= self.x{i}(3);
                temp_v_L = con(1);
                temp_omega_L = con(2);
                temp_theta_Le = st(3);
                temp_theta_r = reference_trajectory(3,i);
                temp_v_r = reference_trajectory(4,i);
                temp_omega_r = reference_trajectory(5,i);
                delt = temp_theta_r - temp_theta_L;
                u_Le = [xd(1) * temp_omega_r * sin(delt) + xd(2) * temp_omega_r* cos(delt)+temp_v_r*cos(delt)-temp_v_L;
                     temp_omega_r-temp_omega_L]; % 获得控制变量的误差
         
                self.costs = self.costs + mtimes(u_Le',...
                    mtimes(param.mat_R, u_Le));
            end
        end
        % 计算上一轮控制量和这一轮第一步控制量的代价值
        function add_prev_input_cost(self, param)
            self.costs = self.costs + mtimes((self.variables_u(:, 1) - self.state.u)',...
                mtimes(param.mat_Rold, self.variables_u(:, 1) - self.state.u));
        end
        % 计算本轮控制量差值的代价值
        function add_input_smoothness_cost(self, param)
            import casadi.*
            for i = 1:(param.horizon - 1)
                self.costs = self.costs + mtimes(...
                    (self.variables_u(:, i+1) - self.variables_u(:, i))',...
                    mtimes(param.mat_dR, self.variables_u(:, i+1) - self.variables_u(:, i)));
            end
        end
        % 静态障碍物
        function add_convex_to_convex_constraint(self, system, param,obs_geo)
            [A_O,B_O]=GeometryUtils.get_polygon_inequation(obs_geo);%获得障碍物不等式参数
            [X_v]=GeometryUtils.get_robot_polygon_vertex(self.x{1},system.x_Rd);% 获得小车的各个顶点
            [A_R,B_R]=GeometryUtils.get_polygon_inequation(X_v);%获得机器人不等式参数
            [cbf_current, lambda_O_current, lambda_R_current] = ...
                GeometryUtils.get_dist_region_to_region(A_O,B_O,A_R,B_R);
            self.obs_distance = [self.obs_distance;cbf_current];
           % 如果和这个障碍物的距离
            if cbf_current > param.safe_distance
               return 
            end
            lambda_O=self.opti.variable(length(B_O), param.horizon_dcbf_obs);
            lambda_R=self.opti.variable(length(B_R), param.horizon_dcbf_obs);
            omega=self.opti.variable(param.horizon_dcbf_obs);
            for i= 1:param.horizon_dcbf_obs
                temp_robot_geo = self.x{i+1};
                [X_v]=GeometryUtils.get_robot_polygon_vertex(temp_robot_geo,system.x_Rd);% 获得小车的各个顶点
                [A_R,B_R]=GeometryUtils.get_polygon_inequation(X_v);%获得机器人不等式参数
                self.opti.subject_to(lambda_O(:,i)>=0);
                self.opti.subject_to(lambda_R(:,i)>=0);
                self.opti.subject_to(...
                    omega(i)*param.gamma_obs^i*(cbf_current-param.margin_distance)...
                    <=-lambda_O(:,i)'* B_O - lambda_R(:,i)' * B_R-param.margin_distance...
                    );
                self.opti.subject_to(...
                    lambda_O(:,i)'*A_O+lambda_R(:,i)'*A_R==0 ...
                    );
                self.opti.subject_to(...
                    lambda_O(:,i)'*A_O*(lambda_O(:,i)'*A_O)'<=1 ...
                    );
                self.opti.subject_to(omega(i)>=0);
                self.costs = self.costs+param.pomega * (omega(i)-1)^2;
                % 热启动
                self.opti.set_initial(lambda_O(:,i), lambda_O_current);
                self.opti.set_initial(lambda_R(:,i), lambda_R_current);
                self.opti.set_initial(omega(i), 0.1);
            end
        end
        % 机器人之间避障
        function add_convex_to_robots_constraint(self, system, param,robots_geo)
            [X_v]=GeometryUtils.get_robot_polygon_vertex(robots_geo(:,1),system.x_Rd);% 获得障碍机器人的各个顶点
            [A_O,B_O]=GeometryUtils.get_polygon_inequation(X_v);%获得障碍机器人不等式参数
            [X_v]=GeometryUtils.get_robot_polygon_vertex(self.x{1},system.x_Rd);% 获得小车的各个顶点
            [A_R,B_R]=GeometryUtils.get_polygon_inequation(X_v);%获得机器人不等式参数
            [cbf_current, lambda_O_current, lambda_R_current] = ...
                GeometryUtils.get_dist_region_to_region(A_O,B_O,A_R,B_R);

           % 如果和这个障碍物的距离
            if cbf_current > param.safe_distance
               return 
            end
            lambda_O=self.opti.variable(length(B_O), param.horizon_dcbf_robot);
            lambda_R=self.opti.variable(length(B_R), param.horizon_dcbf_robot);
            omega=self.opti.variable(param.horizon_dcbf_robot);
            for i= 1:param.horizon_dcbf_robot
                temp_robot_geo = self.x{i+1};
                [X_v]=GeometryUtils.get_robot_polygon_vertex(robots_geo(:,i+1),system.x_Rd);% 获得障碍机器人的各个顶点
                [A_O,B_O]=GeometryUtils.get_polygon_inequation(X_v);%获得障碍机器人不等式参数
                [X_v]=GeometryUtils.get_robot_polygon_vertex(temp_robot_geo,system.x_Rd);% 获得小车的各个顶点
                [A_R,B_R]=GeometryUtils.get_polygon_inequation(X_v);%获得机器人不等式参数
                self.opti.subject_to(lambda_O(:,i)>=0);
                self.opti.subject_to(lambda_R(:,i)>=0);
                
                    %% DC       
%                 self.opti.subject_to(...           
%                     0<=-lambda_O(:,i)'* B_O - lambda_R(:,i)' * B_R-param.margin_distance...
%                     );
                
   %%dclf
                self.opti.subject_to(...
                    omega(i)*param.gamma_robot^i*(cbf_current-param.margin_distance)...
                    <=-lambda_O(:,i)'* B_O - lambda_R(:,i)' * B_R-param.margin_distance...
                    );
                self.opti.subject_to(...
                    lambda_O(:,i)'*A_O+lambda_R(:,i)'*A_R==0 ...
                    );
                self.opti.subject_to(...
                    lambda_O(:,i)'*A_O*(lambda_O(:,i)'*A_O)'<=1 ...
                    );
                self.opti.subject_to(omega(i)>=0);
                self.costs = self.costs+param.pomega * (omega(i)-1)^2;
                % 热启动
                self.opti.set_initial(lambda_O(:,i), lambda_O_current);
                self.opti.set_initial(lambda_R(:,i), lambda_R_current);
                self.opti.set_initial(omega(i), 0.1);
            end
        end
        function add_obstacle_advoidance_constraint(self, system, param, obstacles_geo)
            for i=1 : length(obstacles_geo)
                obs_geo = obstacles_geo{i};
                add_convex_to_convex_constraint(self,system,  param,obs_geo)
            end
        end
        
        function add_robots_advoidance_constraint(self, system, param, robots_geo)
            for i=1 : length(robots_geo)
                obs_geo = robots_geo{i};
%                 param.gamma = 0.5;
                add_convex_to_robots_constraint(self,system, param,obs_geo)
            end
        end
        
        % 热启动
        function add_warm_start(self, last_x, last_u)
            self.opti.set_initial(self.variables_xe, last_x)
            self.opti.set_initial(self.variables_u, last_u)
        end
        
        function setup_L(self, system, param, reference_trajectory, obstacles, last_x, last_u)
            import casadi.*
            self.set_state(system.state);
            self.opti = Opti();
            self.initialize_variables(param);
            self.add_initial_condition_constraint();
            self.add_input_constraint(param);
            % self.add_input_derivative_constraint(param)
            self.get_predict_states(param);
            add_leader_error_dynamics_constraint(self, param, reference_trajectory);
            self.add_reference_trajectory_tracking_cost(param);
            self.add_input_stage_cost_leader(param,reference_trajectory);
            self.add_prev_input_cost(param);
            self.add_input_smoothness_cost(param);
            self.add_obstacle_advoidance_constraint(system,param, obstacles);
            self.add_warm_start(last_x, last_u);
        end
        
        function setup_f(self, system, param, reference_trajectory, obstacles, last_x, last_u, other_robots)
            import casadi.*
            self.set_state(system.state);
            self.opti = Opti();
            self.initialize_variables(param);
            self.add_initial_condition_constraint();
            self.add_input_constraint(param);
            % self.add_input_derivative_constraint(param)
            self.get_predict_states(param);
            add_leader_error_dynamics_constraint(self, param, reference_trajectory);
            self.add_reference_trajectory_tracking_cost(param);
            self.add_input_stage_cost_leader(param,reference_trajectory);
            self.add_prev_input_cost(param);
            self.add_input_smoothness_cost(param);
%             robots = {reference_trajectory(1:3,:)};
            self.add_robots_advoidance_constraint(system,param, other_robots);
            self.add_obstacle_advoidance_constraint(system,param, obstacles);
            self.add_warm_start(last_x, last_u);
        end
        
        function solve=solve_nlp(self, param)
            import casadi.*
            self.opti.minimize(self.costs)
       
            option = struct;
            option.ipopt.max_iter = 100;
            option.ipopt.print_level =0;%0,3
            option.print_time = 0;
            option.ipopt.acceptable_tol =1e-8;
            option.ipopt.acceptable_obj_change_tol = 1e-6;
            tic
            self.opti.solver('ipopt', option)
            try
                opt_sol = self.opti.solve();
            catch
                solve.state = -1;
                return
            end
            solve.t = toc;
            solve.x = zeros(2, param.horizon+1);
            solve.x(:,1) = self.x{1};
            for i = 2:length(self.x)
                solve.x(:,i) = opt_sol.value(self.x{i}); 
            end
            
            solve.xe=opt_sol.value(self.variables_xe);
            solve.u=opt_sol.value(self.variables_u);
            solve.state = 0;
            
        end
    end  
end
