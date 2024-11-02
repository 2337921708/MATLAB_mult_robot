classdef SimRobot < handle
    properties
       warm_start_xe
       warm_start_u
       robot_system
       debuf_info
       solve_last
    end
    
    methods
        function self = SimRobot(warm_start_xe, warm_start_u,robot_system, debuf_info)
            self.warm_start_xe = warm_start_xe;
            self.warm_start_u = warm_start_u;
            self.robot_system = robot_system;
            self.debuf_info = debuf_info;
            self.solve_last = struct;
        end
        function  x_next = runge_kutta_simulation(self,f,x,u,dt)
            k1 = f(u);
            k2 = f(u);
            k3 = f(u);
            k4 = f(u);
            x_next = x + dt/6*(k1+2*k2+2*k3+k4);
        end
        
        function predict_state = sim_leader(self, T, ref_state_pre,param,x_obs)
            % 领航者求解mpc
            mpcOptimization = MpcOptimization(...
                KinematicOmniDynamics.forward_dynamics_opt(T),...
                KinematicOmniDynamics.forward_error_dynamics_opt(T)...
                );
            self.robot_system.state.xe = KinematicOmniDynamics. ...
                get_Leader_initial_state(ref_state_pre(1:3, 1), self.robot_system.state.x);
            mpcOptimization.set_state(self.robot_system.state);
            mpcOptimization.setup_L(self.robot_system,param, ref_state_pre, x_obs, ...
                self.warm_start_xe, self.warm_start_u)
            solve=mpcOptimization.solve_nlp(param);
            self.debuf_info.solve_time(end+1, 1)=solve.t;
            self.debuf_info.travel_trajectory(:,end+1) = self.robot_system.state.x;
            self.debuf_info.real_control(:,end+1)= solve.u(:,1);
            self.robot_system.state.u = solve.u(:,1);
            self.debuf_info.predict_trajectory{end+1,1} = solve.x;
            self.debuf_info.obs_distance(:,end+1) = mpcOptimization.obs_distance;      
            f = @KinematicOmniDynamics.dot_dynamics;
            self.robot_system.state.x = self.runge_kutta_simulation(...
                f, self.robot_system.state.x, self.robot_system.state.u, T...
                );         
            self.warm_start_xe = solve.xe;
            self.warm_start_u = solve.u;
            predict_state = [solve.x(:,1:end-1);solve.u];
            
        end
        
        function predict_state = sim_follower(self, T, ref_state_pre,param,x_obs, other_robots)
            % 领航者求解mpc
            mpcOptimization = MpcOptimization(...
                KinematicOmniDynamics.forward_dynamics_opt(T),...
                KinematicOmniDynamics.forward_error_dynamics_opt(T)...
                );
            self.robot_system.state.xe = KinematicOmniDynamics. ...
                get_Follower_initial_state(ref_state_pre(1:3, 1), ...
                self.robot_system.state.x, self.robot_system.xd);
            mpcOptimization.set_state(self.robot_system.state);
            mpcOptimization.setup_f(self.robot_system,param, ref_state_pre, x_obs, ...
                self.warm_start_xe, self.warm_start_u, other_robots)
            solve=mpcOptimization.solve_nlp(param);
            % 当求解出问题时使用上一次的解
            if solve.state == -1
                solve=self.solve_last;
            end
            self.debuf_info.solve_time(end+1, 1)=solve.t;
            self.debuf_info.travel_trajectory(:,end+1) = self.robot_system.state.x;
            self.debuf_info.real_control(:,end+1)= solve.u(:,1);
            self.robot_system.state.u = solve.u(:,1);
            self.debuf_info.predict_trajectory{end+1,1} = solve.x;
            self.debuf_info.obs_distance(:,end+1) = mpcOptimization.obs_distance;
            f = @KinematicOmniDynamics.dot_dynamics;
            self.robot_system.state.x = self.runge_kutta_simulation(...
                f, self.robot_system.state.x, self.robot_system.state.u, T...
                );
            self.warm_start_xe = solve.xe;
            self.warm_start_u = solve.u;
            predict_state = [solve.x(:,1:end-1);solve.u];
            self.solve_last = solve;
        end
        
        
    end
end