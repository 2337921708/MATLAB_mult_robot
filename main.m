clear;clc;close all;

param = struct;
param.horizon = 8;      %预测时域
param.horizon_dcbf_obs = 3;     %车辆与障碍物之间的避障
param.horizon_dcbf_robot = 1;   %车辆之间避障时域

param.vx_max = 1;       %x轴速度上限
param.vx_min = -1;
param.vy_max = 1;       %y轴速度上限
param.vy_min = -1;
param.vx_dot_max = 0.2; %x轴加速度上限
param.vx_dot_min = -param.vx_dot_max;
param.vy_dot_max=0.2;   %y轴加速度上限
param.vy_dot_min=-param.vy_dot_max;

param.mat_Q = [50,  0;      %过程权重矩阵
                0, 50];
param.terminal_Q = [50,  0; %终端权重矩阵
                     0, 50];
param.mat_R = [1, 0;
               0, 0];
param.mat_Rold = [1, 0;
                  0, 1];
param.mat_dR = [1, 0;
                0, 1];

param.pomega = 100;
param.gamma_obs = 0.8;      %beita的取值，障碍物和机器人的
param.gamma_robot = 0.3;    %beita的取值，机和机器人的
param.safe_distance = 0.6;
param.margin_distance = 0.1;
T = 0.08;


%-------------------------直线障碍物---------------------
x_obs{1,1} = [1, 7.05, 4.8,  3;% 表明了障碍物的边界点
              4,   4,  2.8, 2.8];
x_obs{2,1} = [ 3,  10.7, 12.5, 1;
              1.2, 1.2,  0,  0];
x_obs{3,1} = [ 7,  8.5, 9.5, 10.7;
              1.2,  2,   2,  1.2];
x_obs{4,1} = [11, 23, 21, 12.5;
               4,  4,  3,   3];
x_obs{5,1} = [12.5, 16.5, 15, 14;
               3,    3,  2,  2];
x_obs{6,1} = [16.7, 18.8, 21, 23;
                0,  1.4, 1.4, 0];
x_obs{7,1} = [-2, 28, 28,  -2;
               4,  4, 3.6, 3.6];
x_obs{8,1} = [-2,  30, 30, -2;
              0.4, 0.4, 0, 0];
x_obs{9,1} = [-2, -1.6, -1.6, -2;
               4,   4,     0,  0];
x_obs{10,1} = [27.6, 28, 28, 27.6;
                 4,   4,  0,   0];

%-------------------------直线障碍物---------------------

%-------------------------三角形障碍物---------------------
% x_obs{1,1} = [1.4, 1.9, 1.65;
%               5.4, 5.4, 4.95];
% x_obs{2,1} = [7.1, 7.35, 6.85;
%               5.4, 4.95, 4.95];
% x_obs{3,1} = [3.75, 4.25, 4;
%               3.55, 3.55, 3.1];
% x_obs{4,1} = [ 4,  4.25, 3.75;
%               1.6, 1.15, 1.15];
%-------------------------三角形障碍物---------------------

%-------------------------圆形障碍物---------------------
% x_obs{1,1} = [0.6, 1.1, 0.85;
%               5.4, 5.4, 4.95];
% x_obs{2,1} = [9.05, 9.3, 8.8;
%               5.4, 4.95, 4.95];
% x_obs{3,1} = [4.75, 5.25, 5;
%               1.55, 1.55, 1.1];
% x_obs{4,1} = [  5,  5.25, 4.75;
%               -0.4, -0.85, -0.85];
%-------------------------圆形障碍物---------------------
car_num = 8;        %自定义车辆数量        
debuf_info.travel_trajectory = [];  %车辆运行轨迹矩阵
debuf_info.real_control=[];
debuf_info.predict_trajectory=[];   %车辆预测轨迹
debuf_info.obs_distance=[];         %车辆与障碍物距离矩阵
debuf_info.solve_time=[];

debuf_info_centre = debuf_info(1);  %以所有中心点虚构一辆小车

for i = 2 : car_num       %循环定义8个小车
    debuf_info(i,1) = debuf_info(1);
end

robot_system.x_Rd = [ 0.2, 0.2, -0.2, -0.2;% 小车的各个顶点和小车中心点的向量关系
                     0.17, -0.17, -0.17, 0.17];
robot_system.x_hd = [ 0, 0.2,   0;    % 小车的车头顶点和小车中心点的向量关系（为了画图的时候表明车头）
                     0.15,  0,  -0.15];
robot_system.state = struct;
robot_system.state.x = [0.825;2.5];	%小车初始位置[x; y]
robot_system.xd=[-0.825;-0.5];
robot_system.state.u = [0;0];       %小车初始输入

robot_system_centre = robot_system(1);  %以所有中心点虚构一辆小车

for i = 2 : car_num                 %循环定义8个小车
    robot_system(i,1)= robot_system(1);
end
robot_system_centre.state.x=[0;2];
robot_system_centre.xd=[0;0];

robot_system(2).state.x=[0.825;1.5];   %依次定义每个小车初始位置与队形中小车的相对位置
robot_system(2).xd=[-0.825;0.5];     %小车相对被跟随者的位置[x; y](此处小车车头方向为x)
robot_system(3).state.x=[0.275;2.5];
robot_system(3).xd=[-0.275;-0.5];
robot_system(4).state.x=[0.275;1.5];
robot_system(4).xd=[-0.275;0.5];
robot_system(5).state.x=[-0.275;2.5];
robot_system(5).xd=[0.275;-0.5];
robot_system(6).state.x=[-0.275;1.5];
robot_system(6).xd=[0.275;0.5];
robot_system(7).state.x=[-0.825;2.5];
robot_system(7).xd=[0.825;-0.5];
robot_system(8).state.x=[-0.825;1.5];
robot_system(8).xd=[0.825;0.5];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 在这里区分三个小车的参数，这个地方是实际计算的时候把头车看作一个大三角，并且
% 针对这个大三角进行避障操作。对另外两个小车不采取避障。
% 花图的时候还是画原本小车的大小
% 设置中心小车的大小范围包括所有的车,这个向量设置是把三角形的头顶点设置为小车的参考点
robot_system_centre.x_Rd = [1.025,  1.025,  -1.025, -1.025;     %长方形编队避障
                            0.67,-0.67, -0.67, 0.67];
                        
% robot_system_centre.x_Rd = [0.65,  0.65,  -0.65, -0.65;       %正方形编队避障
%                             0.625,-0.625, -0.625, 0.625];
param(2)=param(1);

% 把跟随者的车的这个避障时域给取消掉
param(2).horizon_dcbf_obs=0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

warm_start_xe = zeros(2, param(1).horizon+1);
warm_start_u = zeros(2, param(1).horizon);

speed = 0.6;            %设置各路径点初速度
path_size_scale = 1;    %对路径进行等比例缩放





sim_tim = 50;          %定义仿真时间，决定小车是否能走完全程
ref_state_pre = zeros(4,param(1).horizon); % 传进控制器的参考轨迹，输入预测时域N个参考就行

environment = Environment_omni(speed, path_size_scale); %环境类实例化
ref_state = environment.calculate_ref_state(T);         %获取参考路径7种状态
ref_state=ref_state';           %参考矩阵转置
ref_state(3:4, end)=[0;0];      %把末端点参考速度变为0(3：角度；4：速度)
xs = ref_state(1:2,end);        %获取终点的x,y
mpciter = 0;
length_ref = size(ref_state,2); %矩阵长度
curent_index=0;

sim_leader = SimRobot(warm_start_xe, warm_start_u,robot_system_centre, debuf_info(1));

for i = 1 : car_num
   sim_follower(i) = SimRobot(warm_start_xe, warm_start_u,robot_system(i), debuf_info(i)); 
end

while(norm((robot_system_centre.state.x-xs),2) > 1e-1 && mpciter < sim_tim / T)
    tic
    % 找到领导者距离参考轨迹最近的参考点
%     distance = vecnorm(ref_state(1:2, :) - robot_system(1).state.x(1:2));
%     [~, curent_index] = min(distance);
%     state_error = KinematicCarDynamics. ...
%                 get_Leader_initial_state(ref_state_pre(1:3, 1),robot_system(1).state.x);
%     % 防止小车停止 
%     if state_error(1) < 0
%         curent_index=curent_index+1;
%     end
    curent_index = curent_index+1;
    if curent_index>length_ref
        curent_index=length_ref;
    end
    if curent_index+param(1).horizon > length_ref
        ref_state_pre(:,1:length_ref-curent_index+1) = ref_state(1:4,curent_index:length_ref);% 取出预测时域内的参考
        temp = repmat(ref_state(1:4,end),1,curent_index+param(1).horizon-length_ref-1);
        ref_state_pre(:,length_ref-curent_index+2:end)=temp;
    else
       ref_state_pre = ref_state(1:4, curent_index:curent_index+param(1).horizon-1);% 取出预测时域内的参考 
    end
    
    leader_predict = sim_leader.sim_leader(T, ref_state_pre,param(1),x_obs);
    %1号跟随者
    other_robots = {[sim_follower(2).robot_system.state.x,sim_follower(2).robot_system.state.x];...
                    [sim_follower(3).robot_system.state.x,sim_follower(3).robot_system.state.x]};
    sim_follower(1).sim_follower(T,leader_predict,param(2),x_obs, other_robots);
    %2号跟随者
    other_robots = {[sim_follower(1).robot_system.state.x,sim_follower(1).robot_system.state.x];...
                    [sim_follower(4).robot_system.state.x,sim_follower(4).robot_system.state.x]};
    sim_follower(2).sim_follower(T,leader_predict,param(2),x_obs, other_robots);
    %3号跟随者
    other_robots = {[sim_follower(1).robot_system.state.x,sim_follower(1).robot_system.state.x];...
                    [sim_follower(4).robot_system.state.x,sim_follower(4).robot_system.state.x];...
                    [sim_follower(5).robot_system.state.x,sim_follower(5).robot_system.state.x]};
    sim_follower(3).sim_follower(T,leader_predict,param(2),x_obs, other_robots);
    %4号跟随者
    other_robots = {[sim_follower(2).robot_system.state.x,sim_follower(2).robot_system.state.x];...
                    [sim_follower(3).robot_system.state.x,sim_follower(3).robot_system.state.x];...
                    [sim_follower(6).robot_system.state.x,sim_follower(6).robot_system.state.x]};
    sim_follower(4).sim_follower(T,leader_predict,param(2),x_obs, other_robots);
    %5号跟随者
    other_robots = {[sim_follower(3).robot_system.state.x,sim_follower(3).robot_system.state.x];...
                    [sim_follower(6).robot_system.state.x,sim_follower(6).robot_system.state.x];...
                    [sim_follower(7).robot_system.state.x,sim_follower(7).robot_system.state.x]};
    sim_follower(5).sim_follower(T,leader_predict,param(2),x_obs, other_robots);
    %6号跟随者
    other_robots = {[sim_follower(4).robot_system.state.x,sim_follower(4).robot_system.state.x];...
                    [sim_follower(5).robot_system.state.x,sim_follower(5).robot_system.state.x];...
                    [sim_follower(8).robot_system.state.x,sim_follower(8).robot_system.state.x]};
    sim_follower(6).sim_follower(T,leader_predict,param(2),x_obs, other_robots);
    %7号跟随者
    other_robots = {[sim_follower(5).robot_system.state.x,sim_follower(5).robot_system.state.x];...
                    [sim_follower(8).robot_system.state.x,sim_follower(8).robot_system.state.x]};
    sim_follower(7).sim_follower(T,leader_predict,param(2),x_obs, other_robots);
    %8号跟随者
    other_robots = {[sim_follower(6).robot_system.state.x,sim_follower(6).robot_system.state.x];...
                    [sim_follower(7).robot_system.state.x,sim_follower(7).robot_system.state.x]};
    sim_follower(8).sim_follower(T,leader_predict,param(2),x_obs, other_robots);
    
    robot_system_centre = sim_leader.robot_system;
    
    mpciter
    if mpciter==73
%        break; 
      a=1;
    end
    mpciter = mpciter + 1;
end


debuf_info_centre =  sim_leader.debuf_info; %中心点信息
for i = 1 : car_num
    debuf_info(i) =  sim_follower(i).debuf_info;
end

robot_system_centre = sim_leader.robot_system;
for i = 1 : car_num
    robot_system(i) = sim_follower(i).robot_system;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 在画图的时候把一些参数给改了
% 设置1小车的大小为原始大小
%robot_system(1).x_Rd = [0.15,0.15,-0.15,-0.15;
%        0.1 ,-0.1,-0.1 ,0.1];% 小车的各个顶点和小车中心点的向量关系

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

draw_multiple_vehocles= DrawMultipleVehicles(ref_state,xs, x_obs);
draw_multiple_vehocles.set_figure_param();
draw_multiple_vehocles.draw_multi_robot(T, robot_system, debuf_info, robot_system_centre, debuf_info_centre);
% draw_multiple_vehocles.draw_obs_distance(T, debuf_info);
% draw_multiple_vehocles.draw_solve_time(debuf_info);
% draw_multiple_vehocles.draw_input(T,debuf_info, param(1));
% draw_multiple_vehocles.draw_distance_robot(T,debuf_info, [robot_system(1).xd, robot_system(2).xd, robot_system(3).xd, robot_system(4).xd, robot_system(5).xd, robot_system(6).xd, robot_system(7).xd, robot_system(8).xd]');
draw_multiple_vehocles.draw_centre_error(T,debuf_info_centre);
draw_multiple_vehocles.draw_all_car_error(T,robot_system,debuf_info,debuf_info_centre);
draw_multiple_vehocles.draw_min_obs_distance(T, debuf_info);

