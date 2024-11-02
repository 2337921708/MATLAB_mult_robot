classdef DrawMultipleVehicles < handle
    properties
        fig
        fontsize_labels
        xs
        ref
        obs
    end

    methods
        
        function self = DrawMultipleVehicles(ref,xs, obs)
            close all;
            self.fig = figure();
            self.fontsize_labels = 14;
            self.ref = ref;
            self.xs=xs;
            self.obs=obs;
        end
        %% 绘制障碍物
        function draw_obs(self)
            for i = 1:length(self.obs)
                hold on
              %  fill(,'r','EdgeColor','r');
                fill(self.obs{i}(1,:),self.obs{i}(2,:),[0.65 0.65 0.65],'EdgeColor',[0.65 0.65 0.65]);
            end
        end
        
        function draw_ref(self)
            % 画出参考轨迹
            hold;
            plot(self.ref(1,:), self.ref(2,:),'--')
        end
 
        function set_figure_param(self)
            set(0,'DefaultAxesFontName', 'Times New Roman')
            set(0,'DefaultAxesFontSize', 12)
            % Animate the robot motion
            %figure;%('Position',[200 200 1280 720]);
            set(self.fig,'PaperPositionMode','auto')
            set(self.fig, 'Color', 'w');
            set(self.fig,'Units','normalized','OuterPosition',[0 0 0.55 1]);
        end
        
        %%
        function draw_robot_outline(self, robot_system, robot_position, car_num)
            % 画机器人轮廓
            if car_num==1 || car_num==2
                temp=GeometryUtils.get_robot_polygon_vertex(robot_position,robot_system.x_Rd);% 获得小车的各个顶点
                hold on
                fill(temp(1,:),temp(2,:),[0.173,0.627,0.173],'EdgeColor',[0.173,0.627,0.173]);
                % 画车头
%                 temp=GeometryUtils.get_robot_polygon_vertex(robot_position,robot_system.x_hd);% 获得小车的各个顶点
%                 hold on
%                 fill(temp(1,:),temp(2,:),'c');
            end
            if car_num==3 || car_num==4
                temp=GeometryUtils.get_robot_polygon_vertex(robot_position,robot_system.x_Rd);% 获得小车的各个顶点
                hold on
                fill(temp(1,:),temp(2,:),[0.580,0.404,0.741],'EdgeColor',[0.580,0.404,0.741]);
                % 画车头
%                 temp=GeometryUtils.get_robot_polygon_vertex(robot_position,robot_system.x_hd);% 获得小车的各个顶点
%                 hold on
%                 fill(temp(1,:),temp(2,:),'c');
            end
            if car_num==5 || car_num==6
                temp=GeometryUtils.get_robot_polygon_vertex(robot_position,robot_system.x_Rd);% 获得小车的各个顶点
                hold on
                fill(temp(1,:),temp(2,:),[1.0,0.494,0.055],'EdgeColor',[1.0,0.494,0.055]);
                % 画车头
%                 temp=GeometryUtils.get_robot_polygon_vertex(robot_position,robot_system.x_hd);% 获得小车的各个顶点
%                 hold on
%                 fill(temp(1,:),temp(2,:),'c');
            end
            if car_num==7 || car_num==8
                temp=GeometryUtils.get_robot_polygon_vertex(robot_position,robot_system.x_Rd);% 获得小车的各个顶点
                hold on
                fill(temp(1,:),temp(2,:),[0.835,0.153,0.153],'EdgeColor',[0.835,0.153,0.153]);
                % 画车头
%                 temp=GeometryUtils.get_robot_polygon_vertex(robot_position,robot_system.x_hd);% 获得小车的各个顶点
%                 hold on
%                 fill(temp(1,:),temp(2,:),'c');
            end
        end
        
        function draw_robot_big_outline(self, robot_system, robot_position)
            % 画全部机器人外侧轮廓
            temp = GeometryUtils.get_robot_polygon_vertex(robot_position,robot_system.x_Rd);% 获得小车的各个顶点
            
            plot_num = length(temp);
            for i = 1:plot_num-1
                hold on
                plot([temp(1,i), temp(1,i+1)],[temp(2,i), temp(2,i+1)],'--r');
            end
            plot([temp(1,plot_num), temp(1,1)],[temp(2,plot_num), temp(2,1)],'--r');
            %fill(temp(1,:),temp(2,:),'y');
        end

        %% 画起点和终点
        function draw_destination(self, robot_system)   %xs是终点坐标
            
            %temp=GeometryUtils.get_robot_polygon_vertex(self.xs,robot_system.x_Rd);% 获得小车的各个顶点
            r=0.05; theta=0:pi/100:2*pi;
            x=r*cos(theta); y=r*sin(theta);
            hold on
            fill(x+self.ref(1,1),y+self.ref(2,1),'k')   %起点 
            hold on
            fill(x+self.xs(1),y+self.xs(2),'k')   %终点
            
            %fill(temp(1,:),temp(2,:),'y');
        end
       
        %% 画预测轨迹
        function draw_predict_trajectory(self, robot_system,predict_trajectory)
            N = size(predict_trajectory, 2);
            X_v = zeros(size(robot_system.x_Rd, 1), size(robot_system.x_Rd, 2)+1);
            temp_x = zeros(size(X_v, 2), N);
            temp_y = zeros(size(X_v, 2), N);
            for i=1:N
                X_v(:,1:end-1) = GeometryUtils.get_robot_polygon_vertex(predict_trajectory(:,i),robot_system.x_Rd);
                X_v(:,end)=X_v(:,1);
                temp_x(:,i)=X_v(1,:)';
                temp_y(:,i)=X_v(2,:)';
            end
            hold on
            plot(temp_x, temp_y,'--r');
        end
        
        %% 画经过轨迹
        function draw_travel_trajectory(self, travel_trajectory, car_num)
            if car_num==1 || car_num==2
                hold on
                plot(travel_trajectory(1,:), travel_trajectory(2,:),'Color',[0.173,0.627,0.173],'linewidth',1.4);
            end
            if car_num==3 || car_num==4
                hold on
                plot(travel_trajectory(1,:), travel_trajectory(2,:),'Color',[0.580,0.404,0.741],'linewidth',1.4);
            end
            if car_num==5 || car_num==6
                hold on
                plot(travel_trajectory(1,:), travel_trajectory(2,:),'Color',[1.0,0.494,0.055],'linewidth',1.4);
            end
            if car_num==7 || car_num==8
                hold on
                plot(travel_trajectory(1,:), travel_trajectory(2,:),'Color',[0.835,0.153,0.153],'linewidth',1.4);
            end
        end
        %% 画出多个车的连线
        % robots_pose三个车的位置
        function draw_attachment_among_cars(self,robots_pose)
            hold on
            plot([robots_pose(1, :),robots_pose(1, 1)],[robots_pose(2, :), robots_pose(2, 1)],'k-', 'Linewidth', 3)     
        end
            

%         %%
%         function draw_all(self, obs, ref)   %该程序没用到
%             num_travel_traectory = size(self.debuf_info.travel_trajectory,2);
%             F = repmat(getframe(gcf), num_travel_traectory-1, 1);
%             for i = 1: num_travel_traectory-1
%                 self.draw_obs(obs);
%                 self.draw_ref(ref);
%                 self.draw_robot_outline(self.debuf_info.travel_trajectory(:,i));
%                 self.draw_destination();
%                 %self.draw_predict_trajectory(self.debuf_info.predict_trajectory{i});
%                 self.draw_travel_trajectory(self.debuf_info.travel_trajectory(:,1:i))
%                 ylabel('$y$-position (m)','interpreter','latex','FontSize',self.fontsize_labels)
%                 xlabel('$x$-position (m)','interpreter','latex','FontSize',self.fontsize_labels)
% %                 axis([-2 11 0 13]); %三角形轨迹地图
% %                 axis([-2 12 -2 12]); %圆形轨迹地图
%                 axis([-2 28 0 4])   %直线轨迹地图
%                 pause(0.1)
%                 box on;
%                 grid on
%                 drawnow
%                 % for video generation
%                 F(i) = getframe(gcf); % to get the current frame   
%             end
%             close(gcf)
%             video = VideoWriter('exp.avi','Motion JPEG AVI');
%             video.FrameRate = 5;  % (frames per second) this number depends on the sampling time and the number of frames you have
%             open(video)
%             writeVideo(video,F)
%             close (video)
%             fullpath = pwd;     %获取工作路径
%             movefile([fullpath, '\exp.avi'], [fullpath, '\Runing_result\exp.avi']); %将结果移到Runing_result
%         end
        %% 画出多个机器人
        function draw_multi_robot(self, T, robot_system, debuf_info, robot_system_centre, debuf_info_centre)
            num_travel_traectory = size(debuf_info(1).travel_trajectory,2);
            F = repmat(getframe(gcf), num_travel_traectory-1, 1);
            num_robot = length(debuf_info);
            robots_pose = zeros(2, num_robot);
            name_index = 1;
            for i = 1: num_travel_traectory-1
                current_time = i*T;
                clf(self.fig);
                self.draw_ref();
                self.draw_obs();
                for k= 1: num_robot
                    %self.draw_robot_big_outline(robot_system_centre, debuf_info_centre.travel_trajectory(:,i));%绘制大框
                    self.draw_robot_outline(robot_system(k), debuf_info(k).travel_trajectory(:,i), k);
                    self.draw_destination(robot_system(k));
                    %self.draw_predict_trajectory(robot_system(k),debuf_info(k).predict_trajectory{i});
                    self.draw_travel_trajectory(debuf_info(k).travel_trajectory(:,1:i), k)
                    robots_pose(:,k)=debuf_info(k).travel_trajectory(:,i);
                end
                %self.draw_attachment_among_cars(robots_pose);
                ylabel('$y$(m)','interpreter','latex','FontSize',self.fontsize_labels)
                xlabel('$x$(m)','interpreter','latex','FontSize',self.fontsize_labels)
                axis equal;
                hold on;
%                 axis([-2 11 0 13]); %三角形轨迹地图
%                 axis([-2 12 -2 12]);%圆形轨迹地图
                axis([-2 28 0 4]);%直线轨迹地图
                pause(0.1)
                box on;
                grid on
                drawnow
                % for video generation
                if current_time == 8 || current_time == 16 || current_time == 24 || current_time == 32 ...
                    || current_time == 43.2 
                    str = ['three_robots_run', num2str(name_index), '.pdf'];
                    exportgraphics(gcf,str,'Resolution',300);
                    name_index = name_index+1;
                    fullpath = pwd;     %获取工作路径
                    movefile([fullpath, '\', str], [fullpath, '\Runing_result\', str]); %将结果移到Runing_result
                end
                F(i) = getframe(gcf); % to get the current frame   
            end
            close(gcf)
            video = VideoWriter('exp.avi','Motion JPEG AVI');
            video.FrameRate = 5;  % (frames per second) this number depends on the sampling time and the number of frames you have
            open(video)
            writeVideo(video,F)
            close (video)
            fullpath = pwd;     %获取工作路径
            movefile([fullpath, '\exp.avi'], [fullpath, '\Runing_result\exp.avi']); %将结果移到Runing_result
        end
        %%
        function draw_obs_distance(self,T, debuf_info)
            
            for k = 1:length(debuf_info)
                t_debuf_info = debuf_info(k);
                figure;
                t = 0:T: T * (size(t_debuf_info.obs_distance, 2)-1);
                num_obs = size(t_debuf_info.obs_distance, 1);
                for i = 1:num_obs
                    subplot(num_obs, 1, i);
                    stairs(t,t_debuf_info.obs_distance(i,:),'k','linewidth',1.5); %axis([0 t(end) -0.75 0.75])
                    str = ['distance between the obstacle ',num2str(i) ,'and the robot' , num2str(k)];
                    title(str);
                    xlabel('t');
                    ylabel('distance(x)');
                end
                str = ['distance_between_the_obstacle_and_the_robot ', num2str(k), '.eps'];
                exportgraphics(gcf,str,'Resolution',300);
                fullpath = pwd;     %获取工作路径
                movefile([fullpath, '\', str], [fullpath, '\Runing_result\', str]); %将结果移到Runing_result
            end
            
        end
        %%
        function draw_solve_time(self,debuf_info)
            figure;
            for k = 1:length(debuf_info)
                subplot(length(debuf_info), 1, k);  
                x = 1:length(debuf_info(k).solve_time);
                x_min=x(1);
                x_max=x(end);
                y_min=min(debuf_info(k).solve_time);
                y_max=max(debuf_info(k).solve_time);
                plot(x,debuf_info(k).solve_time,'b','linewidth',2);
                axis([x_min, x_max, y_min, y_max]);
                xlabel('iterations');
                str = ['solve time' , num2str(k)];
                title(str);    
                ylabel('Time(s)');
            end
            str = 'solve_time.eps';
            exportgraphics(gcf,str,'Resolution',300);
            fullpath = pwd;     %获取工作路径
            movefile([fullpath, '\solve_time.eps'], [fullpath, '\Runing_result\solve_time.eps']);   %将结果移到Runing_result
        end
        %%
        function draw_input(self, T, debuf_info, param)
          figure;
          for k = 1:length(debuf_info)
                subplot(length(debuf_info), 1, k);
                x = 0:T:T*(size(debuf_info(k).real_control,2)-1);
                x_min=x(1);
                x_max=x(end);
                y_min=param.vy_min;
                y_max=param.vy_max;
                plot_v = plot(x,debuf_info(k).real_control(1,:),'b','linewidth',2);
                hold on;
                plot_vmax = plot([x(1), x(end)], [y_max, y_max], 'r--','linewidth',2);
                hold on;
                plot_vmix = plot([x(1), x(end)], [y_min,y_min], 'r--','linewidth',2);
                axis([x_min, x_max, y_min, y_max]);
                str = ['speed of robot' , num2str(k)];
                title(str);
                xlabel('Time(s)');
                ylabel('v(m/s)');
                legend([plot_v plot_vmax plot_vmix],'v','vmax','vmin','location','NorthEast','orientation','horizontal');%图例
          end
          str = 'speed_of_robot.eps';
          exportgraphics(gcf,str,'Resolution',300);
          fullpath = pwd;     %获取工作路径
          movefile([fullpath, '\speed_of_robot.eps'],[fullpath, '\Runing_result\speed_of_robot.eps']);  %将结果移到Runing_result
          figure;
          for k = 1:length(debuf_info)
                subplot(length(debuf_info), 1, k);
                x = 0:T:T*(size(debuf_info(k).real_control, 2)-1);
                x_min=x(1);
                x_max=x(end);
                y_min=param.vx_min;
                y_max=param.vx_max;
                plot_omega = plot(x,debuf_info(k).real_control(2,:),'b','linewidth',2);
                hold on;
                plot_omegamax = plot([x(1), x(end)], [y_max, y_max], 'r--','linewidth',2);
                hold on;
                plot_omegamin = plot([x(1), x(end)], [y_min, y_min], 'r--','linewidth',2);
                axis([x_min, x_max, y_min, y_max]);
                str = ['angular velocity of robot ' , num2str(k)];
                title(str);
                xlabel('Time(s)');
                ylabel('$$\omega$$(rad/s)','Interpreter','latex');  
                legend([plot_omega plot_omegamax plot_omegamin],'$$\omega$$',...
                    '$$\omega$$max','$$\omega$$min','Interpreter','latex','location','NorthEast','orientation','horizontal');%图例
          end
          str = 'angular_velocity_of_robot.eps';
          exportgraphics(gcf,str,'Resolution',300);
          fullpath = pwd;     %获取工作路径
          movefile([fullpath,'\angular_velocity_of_robot.eps'], [fullpath,'\Runing_result\angular_velocity_of_robot.eps']); %将结果移到Runing_result
        end
        %%
        function draw_distance_robot(self, T, debuf_info, xd)
          figure
          debuf_info(end+1)=debuf_info(1);
          xd(end+1, :) = xd(1, :);
          distance_robot = zeros(length(debuf_info)-1, size(debuf_info(1).travel_trajectory, 2));  
          for k = 1:length(debuf_info)-1
                distance_set = norm(xd(k, :),2);
                subplot(length(debuf_info)-1, 1, k);
                for i=1:size(debuf_info(k).travel_trajectory,2)
                    distance_robot(k, i)=norm(debuf_info(k).travel_trajectory(1:2,i)...
                        -debuf_info(k+1).travel_trajectory(1:2,i),2);
                end
                x = 0:T:T*(size(distance_robot, 2)-1);
                x_min=x(1);
                x_max=x(end);
                y_min=min(distance_robot(k,:));
                y_max=max(distance_robot(k,:));
                plot_distance = plot(x, distance_robot(k,:),'b','linewidth',2);
                hold on;
                plot_desired = plot([0,x(end)], [distance_set,distance_set],'r--','linewidth',2);
                axis([x_min, x_max, y_min, y_max]);
                str = ['distance between the robot ' , num2str(k), 'and  the robot', num2str(k+1)];
                title(str);
                xlabel('Time(s)');
                ylabel('$$||d_{ij}||$$','Interpreter','latex'); 
                legend([plot_distance plot_desired],'Relative Distance $$||d_{ij}||$$',...
                    'Desired Distance','Interpreter','latex','location','NorthEast','orientation','horizontal');%图例
          end
          str = 'distance_between_the_robots.jpg';
          exportgraphics(gcf,str,'Resolution',300); 
          fullpath = pwd;     %获取工作路径
          movefile([fullpath,'\distance_between_the_robots.jpg'], [fullpath,'\Runing_result\distance_between_the_robots.jpg']); %将结果移到Runing_result
        end
        
        %% 绘制编队中心点误差
        function draw_centre_error(self,T,debuf_info_centre)
            figure;
            for k = 1:size(debuf_info_centre.travel_trajectory,2)
                centre_error(k) = sqrt((debuf_info_centre.travel_trajectory(1,k)-self.ref(1,k))^2 ...
                                      +(debuf_info_centre.travel_trajectory(2,k)-self.ref(2,k))^2);
            end
            x = 0:T:T*(size(debuf_info_centre.real_control,2)-1);
            plot_centre_error = plot(x,centre_error,"Color",[1,0.498,0.055],'Marker','*','linewidth',2);
            hold on;
        end

        %% 绘制每辆小车的误差
        function draw_all_car_error(self,T,robot_system,debuf_info,debuf_info_centre)
            figure;
            for k = 1:length(debuf_info)
                set_distance(k) = sqrt(robot_system(k).xd(1)^2+robot_system(k).xd(2)^2);
                
                for j = 1:size(debuf_info_centre.travel_trajectory,2)
                    real_distance(k,j) = sqrt((debuf_info(k).travel_trajectory(1,j)-debuf_info_centre.travel_trajectory(1,j))^2 ...
                                             +(debuf_info(k).travel_trajectory(2,j)-debuf_info_centre.travel_trajectory(2,j))^2);
                    all_car_error(k,j) = real_distance(k,j) - set_distance(k);
                end
            end
            
            x = 0:T:T*(size(debuf_info_centre.real_control,2)-1);

            plot_car1_error = plot(x,all_car_error(1,:),"Color",[0.122,0.467,0.706],'Marker','*','linewidth',2);
            hold on;
            plot_car2_error = plot(x,all_car_error(2,:),"Color",[1,0.498,0.055],'Marker','+','linewidth',2);
            hold on;
            plot_car3_error = plot(x,all_car_error(3,:),"Color",[0.173,0.627,0.173],'Marker','x','linewidth',2);
            hold on;
            plot_car4_error = plot(x,all_car_error(4,:),"Color",[0.839,0.153,0.153],'Marker','diamond','linewidth',2);
            hold on;
            plot_car5_error = plot(x,all_car_error(5,:),"Color",[0.580,0.404,0.741],'Marker','hexagram','linewidth',2);
            hold on;
            plot_car6_error = plot(x,all_car_error(6,:),"Color",[0.549,0.337,0.294],'Marker','o','linewidth',2);
            hold on;
            plot_car7_error = plot(x,all_car_error(7,:),"Color",[0.890,0.467,0.761],'Marker','square','linewidth',2);
            hold on;
            plot_car8_error = plot(x,all_car_error(8,:),"Color",[0.498,0.498,0.498],'Marker','pentagram','linewidth',2);
            hold on;

        end
        
        %% 画距离障碍物的最近距离
        function draw_min_obs_distance(self,T, debuf_info)
            figure;
            for i = 1 : size(debuf_info(1).obs_distance,2)
                min_distance_all = inf;
                for j = 1 : length(debuf_info)
                    min_distance = inf;
                    for k = 1 : size(debuf_info(1).obs_distance,1)
                        if debuf_info(j).obs_distance(k,i) < min_distance
                            min_distance = debuf_info(j).obs_distance(k,i);
                        end
                    end
                    if min_distance < min_distance_all
                        min_distance_all = min_distance;
                    end
                end
                obs_distance(i) = min_distance_all;
            end
            x = 0:T:T*(size(debuf_info(1).travel_trajectory,2)-1);
            plot_obs_distance = plot(x,obs_distance(:),"Color",[0.498,0.498,0.498],'linewidth',2);
            legend([plot_obs_distance], 'minObsDistance');%图例
            fullpath = pwd;     %获取工作路径
            savefig([fullpath,'\Runing_result\minObsDistance.fig']);
        end  

    end
    
end
    
    