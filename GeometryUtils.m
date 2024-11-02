classdef GeometryUtils < handle
    methods(Static)
        function [dist, lambda_O, lambda_R] = get_dist_region_to_region(A_O,B_O,A_R,B_R)
            % 变量和代价值
            import casadi.*
            opti = Opti();
            y_O = opti.variable(size(A_O,2), 1);
            y_R = opti.variable(size(A_R,2), 1);
            cost = 0;
            % 约束
            constraint1 = mtimes(A_O, y_O) <= B_O;
            constraint2 = mtimes(A_R, y_R)<=B_R;
            opti.subject_to(constraint1);
            opti.subject_to(constraint2);
            dist_vector = y_O - y_R;
            cost = cost+ mtimes(dist_vector', dist_vector);
            opti.minimize(cost)
            option = struct;
            option.ipopt.max_iter = 100;
            option.ipopt.print_level =0;%0,3
            option.print_time = 0;
            option.ipopt.acceptable_tol =1e-8;
            option.ipopt.acceptable_obj_change_tol = 1e-6;
%             p_opts = struct('expand',true);
%             s_opts = struct('max_iter',100);
            opti.solver('ipopt',option);
            opt_sol = opti.solve();
            dist=opt_sol.value(norm_2(dist_vector));
            if dist > 0
                lambda_O = opt_sol.value(opti.dual(constraint1)) / (2 * dist);
                lambda_R = opt_sol.value(opti.dual(constraint2)) / (2 * dist);
            else
                lambda_O = zeros(length(B_O), 1);
                lambda_R = zeros(length(B_R), 1);
                
            end
        end
        %%
        function [A,B]=get_polygon_inequation(points)
            % points为顺时针的多边形顶点
            num = size(points,2);
            A = [];
            B= [];
            for i=1:num-1
                x1 = points(1,i);
                y1 = points(2,i);
                x2 = points(1,i+1);
                y2 = points(2,i+1);
                a = y1 - y2;
                b = -(x1-x2);
                c = -(x1-x2)*y1+(y1-y2)*x1;
                A=[A;a,b];
                B=[B;c];
            end
            x1 = points(1,num);
            y1 = points(2,num);
            x2 = points(1,1);
            y2 = points(2,1);
            a = y1 - y2;
            b = -(x1-x2);
            c = -(x1-x2)*y1+(y1-y2)*x1;
            A=[A;a,b];
            B=[B;c];
        end
        %% 全向轮的机器人角度都固定为0度
        function vertex =get_robot_polygon_vertex(XR,Xvd)
            hold off;
            % Xvd=[-2,1; 2,1;2,-1;-2,-1]; 为小车的各个顶点与小车中心的关系
            % XR=[2,1,pi/2]; 小车中心坐标
            theta = 0; 
            lon2xy=[cos(theta) -sin(theta);
                sin(theta) cos(theta)];
            Xv = [];
            for i = 1:size(Xvd,2)
                Xv = [Xv,lon2xy*Xvd(:,i)+XR(1:2)];
            end
            % figure(1);
            vertex = Xv;
        end
    end
end