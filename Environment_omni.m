% 全向车的环境配置
classdef Environment_omni < Environment
    methods
        function self=Environment_omni(ref_speed, path_size_scale)
            self = self@Environment(ref_speed, path_size_scale);
        end
        
        function ref_state = calculate_ref_state(self,T)
            IDX_X = 1;
            IDX_Y = 2;
            IDX_Vx= 3;
            IDX_Vy= 4;
            IDX_TIME=5;
            
%-------------直线路径---------------
            point = [0.5, 2;
                     26.5, 2];
%----------------------------------

%-------------三角形路径---------------
%             point = [-1, 2;
%                       4,12;
%                      10, 2
%                      -1, 2];
%-------------------------------------

%-------------圆形路径---------------
%             point = [5, 5;
%                      5, 5];
%------------------------------------

            %self.path_design(point); % using spline
             self.path_linear_design(point);
            %self.path_circle_design(point);
            ref = zeros(length(self.path), 5); % x y vx vy t
            ref(:,IDX_X:IDX_Y) = self.path * self.path_size_scale;
            % 计算路径时间
            for i = 2:length(ref)
                v_ = self.ref_speed;
                d_ = norm(ref(i,IDX_X:IDX_Y)-ref(i-1,IDX_X:IDX_Y));
                dt_ = d_ / v_;
                ref(i, IDX_TIME) = ref(i-1, IDX_TIME) + dt_;
            end
            ref_state = interp1(ref(:, IDX_TIME), ref(:,IDX_X:IDX_Y), 0:T:ref(end, IDX_TIME));
            ref_state = [ref_state, zeros(size(ref_state, 1), 2)];
            % 计算Vx和Vy
            for i = 2:length(ref_state)-1
                x_forward = ref_state(i+1, IDX_X);
                x_backward = ref_state(i-1, IDX_X);
                y_forward = ref_state(i+1, IDX_Y);
                y_backward = ref_state(i-1, IDX_Y);
                yaw = atan2(y_forward-y_backward, x_forward-x_backward);
                ref_state(i, IDX_Vx) = self.ref_speed * cos(yaw);
                ref_state(i, IDX_Vy) = self.ref_speed * sin(yaw);
            end
            ref_state(1, IDX_Vx:IDX_Vy)= ref_state(2, IDX_Vx:IDX_Vy);
            ref_state(end, IDX_Vx:IDX_Vy)= ref_state(end-1, IDX_Vx:IDX_Vy);

            save("refer_path.mat", "ref");% 保存放大后的参考路径
            
        end
        
        %% design path from points by spline
        function path_design(self, point)

            s = 1:1:length(point);
            
            px_spline = spline(s, point(:,1), 1:0.01:length(point));
            py_spline = spline(s, point(:,2), 1:0.01:length(point));           
           %% plot with attitude

            figure(101);
            plot(point(:,1), point(:,2), 'bo-'); hold on;
            plot(px_spline, py_spline,'r-'); grid on; hold off;
            close all;
            %% save path
            self.path = [px_spline', py_spline'];
            
            %         save('path', 'path')
        end
         %% 线性插值设计路径
        function path_linear_design(self, point)
            
            point_num = length(point);
            px_linear = [];
            py_linear = [];
            
            for i = 1:point_num-1
                if point(i, 1) < point(i+1,1)
                    px_linear_temp = point(i,1):0.01:point(i+1,1);
                    py_linear_temp = interp1(point(i:i+1,1), point(i:i+1,2), px_linear_temp, 'linear');
                end
                if point(i, 1) > point(i+1,1)
                    temp = point(i, 1);
                    point(i, 1) = point(i+1,1);
                    point(i+1,1) = temp;
                    px_linear_temp = point(i,1):0.01:point(i+1,1);
                    py_linear_temp = interp1(point(i:i+1,1), point(i:i+1,2), px_linear_temp, 'linear');
                    px_linear_temp = flip(px_linear_temp);
                    py_linear_temp = flip(py_linear_temp);
                    temp = point(i, 1);
                    point(i, 1) = point(i+1,1);
                    point(i+1,1) = temp;
                end
                px_linear_temp(:,1)=[];
                py_linear_temp(:,1)=[];
                px_linear = [px_linear,px_linear_temp];
                py_linear = [py_linear,py_linear_temp];
            end
            
            %% plot with attitude
            
            figure(101);
            plot(point(:,1), point(:,2), 'bo-'); hold on;
            plot(px_linear, py_linear,'r-'); grid on; hold off;
            %close all;
            %% save path
            self.path = [px_linear', py_linear'];
            
            %         save('path', 'path')
        end
        %% 给定轨迹插值设计路径
        function path_circle_design(self, point)
            
            theta = 0:0.01:pi;
            radius = point(2,1);
            px_linear = [];
            py_linear = [];
            
            x_temp = cos(theta(:)) * radius;
            y_temp = sin(theta(:)) * radius;
            px_linear_temp = x_temp + point(1,1);
            py_linear_temp = y_temp + point(1,2);
            px_linear_temp = flip(px_linear_temp);
            py_linear_temp = flip(py_linear_temp);
            px_linear_temp(1)=[];
            py_linear_temp(1)=[];
            px_linear = [px_linear;px_linear_temp];
            py_linear = [py_linear;py_linear_temp];
            
            px_linear_temp = x_temp + point(1,1);
            py_linear_temp = -y_temp + point(1,2);
            px_linear_temp(1)=[];
            py_linear_temp(1)=[];
            px_linear = [px_linear;px_linear_temp];
            py_linear = [py_linear;py_linear_temp];
            
            %% plot with attitude
            
            figure(101);
            plot(point(:,1), point(:,2), 'bo-'); hold on;
            plot(px_linear, py_linear,'r-'); grid on; hold off;
            %close all;
            %% save path
            self.path = [px_linear, py_linear];
            
            %         save('path', 'path')
        end
        
    end
end

