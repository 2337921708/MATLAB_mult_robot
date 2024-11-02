classdef Environment < handle
    properties
        ref_speed
        path_size_scale
        path
    end
    methods
        function self=Environment(ref_speed, path_size_scale)
            self.ref_speed = ref_speed;
            self.path_size_scale = path_size_scale;
            self.path = [];
        end
        
        function ref_state = calculate_ref_state(self,T)
            IDX_X = 1;
            IDX_Y = 2;
            IDX_XY = 1:2;
            IDX_XYYAW = 1:3;
            IDX_YAW = 3;
            IDX_VEL = 4;
            IDX_CURVATURE = 5;
            IDX_W = 6;% 角速度
            IDX_TIME = 7;
            %处理路径
            point = [-1, 1;
                        0,-0.5;
                        3.5,-0.5;
                        3.5,2;
                        4.5,3
                        6.5,2;
                        6.5,-0.5;
                        10,0;
                        10,5];
            self.path_design(point); % using spline
            ref = zeros(length(self.path), 7);
            
            self.path(:,IDX_XY) = self.path(:,IDX_XY) * self.path_size_scale;
            ref(:,IDX_XYYAW) = self.path(:,IDX_XYYAW);
            ref(:,IDX_VEL) = ones(length(self.path),1)*self.ref_speed;
            % 计算路径曲率
            for i = 2:length(ref)-1
                p1_ = ref(i-1,IDX_XY);
                p2_ = ref(i, IDX_XY);
                p3_ = ref(i+1, IDX_XY);
                A_ = ((p2_(1)-p1_(1))*(p3_(2)-p1_(2)) - (p2_(2)-p1_(2))*(p3_(1)-p1_(1))) / 2;
                ref(i, IDX_CURVATURE) = 4 * A_ / (norm(p1_-p2_) * norm(p2_-p3_) * norm(p3_-p1_));
                ref(i, IDX_W) = ref(i, IDX_VEL)* ref(i, IDX_CURVATURE); %汽车角速度等于v乘以曲率
            end
            
            % 计算路径时间
            for i = 2:length(ref)
                v_ = ref(i,IDX_VEL);
                d_ = norm(ref(i,IDX_XY)-ref(i-1,IDX_XY));
                dt_ = d_ / v_;
                ref(i, IDX_TIME) = ref(i-1, IDX_TIME) + dt_;
            end
            save("refer_path.mat", "ref");% 保存放大后的参考路径
            
            ref_state = interp1(ref(:, IDX_TIME), ref(:,1:6), 0:T:ref(end, IDX_TIME));
            ref_state(:,7) = 0:T:ref(end, IDX_TIME);
            
        end
        
        %% 通过样条从点开始设计路径
        function path_design(self, point)
            
            s = 1:1:length(point);
            
            px_spline = spline(s, point(:,1), 1:0.01:length(point));
            py_spline = spline(s, point(:,2), 1:0.01:length(point));           
            %% 插入偏航            
            yaw = zeros(length(px_spline), 1);
            for i = 2:length(px_spline)-1
                x_forward = px_spline(i+1);
                x_backward = px_spline(i-1);
                y_forward = py_spline(i+1);
                y_backward = py_spline(i-1);
                yaw(i) = atan2(y_forward-y_backward, x_forward-x_backward);
            end
            yaw(1) = yaw(2);
            yaw(end) = yaw(end-1);   
            %% plot with attitude
            
            arrow_scale = 0.01;
            
            figure(101);
            plot(point(:,1), point(:,2), 'bo-'); hold on;
            quiver(px_spline', py_spline', cos(yaw)*arrow_scale, sin(yaw)*arrow_scale);
            plot(px_spline, py_spline,'r-'); grid on; hold off;
            close all;
            %% save path
            self.path = [px_spline', py_spline', yaw];
            
            %         save('path', 'path')
        end
    end
end

