% =========================================================================
% SE(2)-DMLKF 算法仿真数据生成脚本 (升级版：8辆车)
% 环境：30m x 50m 无遮挡空间
% 基站：2个，可自定义位置，默认位于 (20,0) 和 (0,40) [支持对角线或其他配置]
% 车辆：8辆，不同起点，运行5分钟(300秒)，速度/加速度提升约15%，轨迹简化(<=1次转弯)
% 采样：dt = 0.5s，每车共 600 个轨迹点
% =========================================================================
clc; clear; close all;

%% 1. 全局参数设置
dt = 0.5;                % 采样时间 0.5秒
anchor_num = 2;
vehicle_num = 8;         % 升级为 8 辆车

% 【自定义基站位置区域】—— 默认保持与原脚本一致，可自由修改
anchors =[30, 0;          % 基站1位置
          0,50];          % 基站2位置

% 设置目标文件夹路径
save_dir = 'E:\SE2_DMLKF_Project\DataGenerator';
trajectories_mat_name = 'Trj_data_Veh8_Anc2.mat';

% 创建总体结构体
trajectories = struct();
IMU_noise_params = struct();
UWB_noise_params = struct();

% 初始速度基准略微提升（原 0.08 -> 现 0.092，提升15%）
v0_new = 0.092; 

% 初始化8辆车的起点、朝向和初始速度
% 环境范围 X: [0, 30], Y: [0, 50]
trajectories.V1 = init_vehicle(4,  2,  pi/2, v0_new);  % 左下角，朝北
trajectories.V2 = init_vehicle(26, 5,  pi/2, v0_new);  % 右下角，朝北
trajectories.V3 = init_vehicle(2,  12, 0,    v0_new);  % 左侧中下，朝东
trajectories.V4 = init_vehicle(3,  48, 0,    v0_new);  % 左侧中上，朝东
trajectories.V5 = init_vehicle(27, 40, pi,   v0_new);  % 右上角，朝西
trajectories.V6 = init_vehicle(17, 2,  pi/2, v0_new);  % 下方中部，朝北
trajectories.V7 = init_vehicle(28, 20, pi,   v0_new);  % 右侧中部，朝西
trajectories.V8 = init_vehicle(2,  28, 0,    v0_new);  % 左侧正中，朝东

%% 2. 拼接生成每辆车的轨迹 (每车仅 1 次转弯，包含加减速，速度提升约15%)
% 采样说明：总持续时间 299.5s，配合初始点共 600 个点。
% 转弯设置：pi/60 rad/s 持续 30s 正好转 90 度。
% 加速度基准：原 0.001 -> 现 0.00115；原 0.0015 -> 现 0.00172（提升15%）

% ---------------- Vehicle 1: 北 -> 东 (右转) ----------------
trajectories.V1 = add_trajectory_segment(trajectories.V1,  0.00115, 0,      40,   dt); % 加速↑
trajectories.V1 = add_trajectory_segment(trajectories.V1,  0,       0,      100,  dt); % 匀速↑
trajectories.V1 = add_trajectory_segment(trajectories.V1, -0.00115, 0,      40,   dt); % 减速↑
trajectories.V1 = add_trajectory_segment(trajectories.V1,  0,      -pi/60,  30,   dt); % 右转90°: N→E
trajectories.V1 = add_trajectory_segment(trajectories.V1,  0.00115, 0,      40,   dt); % 加速→
trajectories.V1 = add_trajectory_segment(trajectories.V1, -0.00115, 0,      49.5, dt); % 减速→

% ---------------- Vehicle 2: 北 -> 西 (左转) ----------------
trajectories.V2 = add_trajectory_segment(trajectories.V2,  0.00172, 0,      30,   dt); % 加速↑
trajectories.V2 = add_trajectory_segment(trajectories.V2,  0,       0,      110,  dt); % 匀速↑
trajectories.V2 = add_trajectory_segment(trajectories.V2, -0.00172, 0,      30,   dt); % 减速↑
trajectories.V2 = add_trajectory_segment(trajectories.V2,  0,       pi/60,  30,   dt); % 左转90°: N→W
trajectories.V2 = add_trajectory_segment(trajectories.V2,  0.00115, 0,      50,   dt); % 加速←
trajectories.V2 = add_trajectory_segment(trajectories.V2, -0.00115, 0,      49.5, dt); % 减速←

% ---------------- Vehicle 3: 东 -> 北 (左转) ----------------
trajectories.V3 = add_trajectory_segment(trajectories.V3,  0.00115, 0,      10,   dt); % 加速→
trajectories.V3 = add_trajectory_segment(trajectories.V3,  0,       0,      90,   dt); % 匀速→
trajectories.V3 = add_trajectory_segment(trajectories.V3, -0.00230, 0,      20,   dt); % 减速→
trajectories.V3 = add_trajectory_segment(trajectories.V3,  0,       pi/60,  30,   dt); % 左转90°: E→N
trajectories.V3 = add_trajectory_segment(trajectories.V3,  0.00115, 0,      60,   dt); % 加速↑
trajectories.V3 = add_trajectory_segment(trajectories.V3, -0.00115, 0,      89.5, dt); % 减速↑

% ---------------- Vehicle 4: 东 -> 南 (右转) ----------------
trajectories.V4 = add_trajectory_segment(trajectories.V4,  0.00115, 0,      40,   dt); % 加速→
trajectories.V4 = add_trajectory_segment(trajectories.V4,  0,       0,      80,   dt); % 匀速→
trajectories.V4 = add_trajectory_segment(trajectories.V4,  0,      -pi/60,  30,   dt); % 右转90°: E→S
trajectories.V4 = add_trajectory_segment(trajectories.V4, -0.00115, 0,      50,   dt); % 减速→
trajectories.V4 = add_trajectory_segment(trajectories.V4,  0.00115, 0,      50,   dt); % 加速↓
trajectories.V4 = add_trajectory_segment(trajectories.V4, -0.00115, 0,      49.5, dt); % 减速↓

% ---------------- Vehicle 5: 西 -> 南 (左转) ----------------
trajectories.V5 = add_trajectory_segment(trajectories.V5,  0.00115, 0,      30,   dt); % 加速←
trajectories.V5 = add_trajectory_segment(trajectories.V5,  0,       0,      100,  dt); % 匀速←
trajectories.V5 = add_trajectory_segment(trajectories.V5,  0,       pi/60,  30,   dt); % 左转90°: W→S
trajectories.V5 = add_trajectory_segment(trajectories.V5, -0.00115, 0,      40,   dt); % 减速←
trajectories.V5 = add_trajectory_segment(trajectories.V5,  0.00115, 0,      50,   dt); % 加速↓
trajectories.V5 = add_trajectory_segment(trajectories.V5, -0.00115, 0,      49.5, dt); % 减速↓

% ---------------- Vehicle 6: 北 -> 东 (右转) ----------------
trajectories.V6 = add_trajectory_segment(trajectories.V6,  0.00150, 0,      40,   dt); % 加速↑
trajectories.V6 = add_trajectory_segment(trajectories.V6,  0,       0,      80,   dt); % 匀速↑
trajectories.V6 = add_trajectory_segment(trajectories.V6, -0.00150, 0,      50,   dt); % 减速↑
trajectories.V6 = add_trajectory_segment(trajectories.V6,  0,       0,      30,   dt); 
trajectories.V6 = add_trajectory_segment(trajectories.V6,  0.00115, 0,      50,   dt); % 加速→
trajectories.V6 = add_trajectory_segment(trajectories.V6, -0.00115, 0,      49.5, dt); % 减速→

% ---------------- Vehicle 7: 西 -> 北 (右转) ----------------
trajectories.V7 = add_trajectory_segment(trajectories.V7,  0.00130, 0,      50,   dt); % 加速←
trajectories.V7 = add_trajectory_segment(trajectories.V7,  0,       0,      70,   dt); % 匀速←
trajectories.V7 = add_trajectory_segment(trajectories.V7, -0.00130, 0,      50,   dt); % 减速←
trajectories.V7 = add_trajectory_segment(trajectories.V7,  0,      -pi/60,  30,   dt); % 右转90°: W→N
trajectories.V7 = add_trajectory_segment(trajectories.V7,  0.00115, 0,      40,   dt); % 加速↑
trajectories.V7 = add_trajectory_segment(trajectories.V7, -0.00115, 0,      59.5, dt); % 减速↑

% ---------------- Vehicle 8: 东 -> 南 (右转) ----------------
trajectories.V8 = add_trajectory_segment(trajectories.V8,  0.00115, 0,      60,   dt); % 加速→
trajectories.V8 = add_trajectory_segment(trajectories.V8,  0,       0,      70,   dt); % 匀速→
trajectories.V8 = add_trajectory_segment(trajectories.V8, -0.00115, 0,      50,   dt); % 减速→
trajectories.V8 = add_trajectory_segment(trajectories.V8,  0,      -pi/60,  30,   dt); % 右转90°: E→S
trajectories.V8 = add_trajectory_segment(trajectories.V8,  0.00150, 0,      40,   dt); % 加速↓
trajectories.V8 = add_trajectory_segment(trajectories.V8, -0.00150, 0,      49.5, dt); % 减速↓

%% 3 生成50Hz的IMU数据 (带随机游走偏置和高斯噪声)
f_imu = 50;                  % IMU频率 50Hz
dt_imu = 1 / f_imu;          % IMU采样间隔 0.02s

IMU_noise_params.sigma_na = 0.05;      
IMU_noise_params.sigma_nw = 0.005;     
IMU_noise_params.sigma_ba = 0.002;     
IMU_noise_params.sigma_bw = 0.0002;    

v_names = {'V1', 'V2', 'V3', 'V4', 'V5', 'V6', 'V7', 'V8'};
for i = 1:vehicle_num
    veh = trajectories.(v_names{i});
    t_traj = veh.Time_true;
    
    t_imu = (t_traj(1) : dt_imu : t_traj(end))';
    N_imu = length(t_imu);
    
    theta_unwrapped = unwrap(veh.Theta_true);
    theta_imu_unwrapped = interp1(t_traj, theta_unwrapped, t_imu, 'linear');
    theta_imu = mod(theta_imu_unwrapped + pi, 2*pi) - pi;
    v_imu_mag = interp1(t_traj, veh.V_true, t_imu, 'linear');
    
    vx_imu = v_imu_mag .* cos(theta_imu);
    vy_imu = v_imu_mag .* sin(theta_imu);
    
    w_true = gradient(theta_imu_unwrapped, dt_imu);
    ax_true = gradient(vx_imu, dt_imu);
    ay_true = gradient(vy_imu, dt_imu);
    
    ab_x_true = ax_true .* cos(theta_imu) + ay_true .* sin(theta_imu);
    ab_y_true = -ax_true .* sin(theta_imu) + ay_true .* cos(theta_imu);
    
    ba_init = (rand(1, 2) - 0.5) * 0.2;  
    bw_init = (rand(1, 1) - 0.5) * 0.02; 
    
    b_a_x = ba_init(1) + cumsum(randn(N_imu, 1) * IMU_noise_params.sigma_ba * sqrt(dt_imu));
    b_a_y = ba_init(2) + cumsum(randn(N_imu, 1) * IMU_noise_params.sigma_ba * sqrt(dt_imu));
    b_w   = bw_init + cumsum(randn(N_imu, 1) * IMU_noise_params.sigma_bw * sqrt(dt_imu));
    
    acc_noise_x = randn(N_imu, 1) * IMU_noise_params.sigma_na;
    acc_noise_y = randn(N_imu, 1) * IMU_noise_params.sigma_na;
    gyro_noise  = randn(N_imu, 1) * IMU_noise_params.sigma_nw;
    
    acc_m_x = ab_x_true + b_a_x + acc_noise_x;
    acc_m_y = ab_y_true + b_a_y + acc_noise_y;
    gyro_m  = w_true + b_w + gyro_noise;
    
    trajectories.(v_names{i}).IMU_Time = t_imu;
    trajectories.(v_names{i}).IMU_acc_m =[acc_m_x, acc_m_y]; 
    trajectories.(v_names{i}).IMU_gyro_m = gyro_m;            
    
    trajectories.(v_names{i}).IMU_bias_a_true =[b_a_x, b_a_y];
    trajectories.(v_names{i}).IMU_bias_w_true = b_w;
end

%% 4. 生成 10Hz 的 UWB 测距数据 (基站测距和相对测距)
f_uwb = 10;                     % UWB频率 10Hz
dt_uwb = 1 / f_uwb;             % UWB采样间隔 0.1s

UWB_noise_params.sigma_anc = 0.06;  
UWB_noise_params.sigma_rel = 0.06; 

t_end_all = trajectories.V1.Time_true(end);
t_uwb = (0 : dt_uwb : t_end_all)';
N_uwb = length(t_uwb);

% 扩展至 8 辆车的动态插值
pos_true_uwb = zeros(N_uwb, 2, vehicle_num);
for i = 1:vehicle_num
    veh = trajectories.(v_names{i});
    pos_true_uwb(:, 1, i) = interp1(veh.Time_true, veh.X_true, t_uwb, 'linear');
    pos_true_uwb(:, 2, i) = interp1(veh.Time_true, veh.Y_true, t_uwb, 'linear');
end

for i = 1:vehicle_num
    v_name = v_names{i};
    
    % ---------------- A. 基站测距 ----------------
    UWB_Anchor = zeros(N_uwb, 1 + anchor_num);
    UWB_Anchor(:, 1) = t_uwb;
    
    for a_idx = 1:size(anchors, 1)
        dx = pos_true_uwb(:, 1, i) - anchors(a_idx, 1);
        dy = pos_true_uwb(:, 2, i) - anchors(a_idx, 2);
        dist_true = sqrt(dx.^2 + dy.^2);
        
        UWB_Anchor(:, 1 + a_idx) = dist_true + randn(N_uwb, 1) * UWB_noise_params.sigma_anc;
    end
    trajectories.(v_name).UWB_Anchor = UWB_Anchor;
    
    % ---------------- B. 相对测距 ----------------
    UWB_Relative = zeros(N_uwb, 1 + vehicle_num);
    UWB_Relative(:, 1) = t_uwb;
    
    for j = 1:vehicle_num
        if i == j
            UWB_Relative(:, 1 + j) = NaN;
        else
            dx = pos_true_uwb(:, 1, i) - pos_true_uwb(:, 1, j);
            dy = pos_true_uwb(:, 2, i) - pos_true_uwb(:, 2, j);
            dist_true = sqrt(dx.^2 + dy.^2);
            
            UWB_Relative(:, 1 + j) = dist_true + randn(N_uwb, 1) * UWB_noise_params.sigma_rel;
        end
    end
    trajectories.(v_name).UWB_Relative = UWB_Relative;
end

%% 5. 结果验证与可视化
figure('Name', 'Multi-Agent UWB/IMU Trajectories (8 Vehicles)', 'Position',[100, 100, 600, 800]);
hold on; grid on; axis equal;

% 适应扩大的 30x50m 场地调整坐标轴限制
xlim([-3, 33]); ylim([-3, 53]);
xlabel('X Position (m)'); ylabel('Y Position (m)');
title('8 Vehicles Trajectories (300s, 30x50m Environment)');

% 绘制边界 (对应新的环境尺寸)
rectangle('Position',[0, 0, 30, 50], 'EdgeColor', 'k', 'LineWidth', 1.5, 'LineStyle', '--');

% 绘制基站
h_anchor = plot(anchors(:,1), anchors(:,2), '^', 'MarkerSize', 12, ...
    'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k', 'DisplayName', 'UWB Anchors');

% 绘制每辆小车的轨迹
colors = lines(vehicle_num);
h_traj = zeros(1, vehicle_num);
for i = 1:vehicle_num
    v_data = trajectories.(v_names{i});
    
    % 输出验证信息到控制台
    fprintf('%s 轨迹点数: %d, IMU数据点数: %d, 最终位置: (%.1f, %.1f)\n', ...
        v_names{i}, length(v_data.Time_true), length(v_data.IMU_Time), v_data.X_true(end), v_data.Y_true(end));
    
    h_traj(i) = plot(v_data.X_true, v_data.Y_true, 'Color', colors(i,:), 'LineWidth', 2, ...
        'DisplayName', sprintf('Vehicle %d', i));
    
    % 绘制起点和终点
    plot(v_data.X_true(1), v_data.Y_true(1), 'o', 'MarkerSize', 6, 'MarkerFaceColor', colors(i,:), 'Color', colors(i,:));
    plot(v_data.X_true(end), v_data.Y_true(end), '*', 'MarkerSize', 8, 'Color', colors(i,:));
end
legend([h_anchor, h_traj], 'Location', 'northeastoutside');
hold off;

%% 6. 保存生成的实验数据到指定目录
% 取消注释以下代码可激活自动保存功能
if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end
save_path = fullfile(save_dir, trajectories_mat_name);
save(save_path, 'trajectories', 'anchors', 'IMU_noise_params', 'UWB_noise_params');
fprintf('\n==== 数据生成完成 ====\n');

%% =========================================================================
% 内部函数定义区域
% =========================================================================
function veh = init_vehicle(x0, y0, theta0, v0)
    veh.X_true = x0;
    veh.Y_true = y0;
    veh.Theta_true = theta0;
    veh.V_true = v0;
    veh.Time_true = 0;
end

function veh = add_trajectory_segment(veh, a, w, duration, dt)
    num_steps = round(duration / dt);
    
    curr_x = veh.X_true(end);
    curr_y = veh.Y_true(end);
    curr_theta = veh.Theta_true(end);
    curr_v = veh.V_true(end);
    curr_time = veh.Time_true(end);
    
    new_X = zeros(num_steps, 1);
    new_Y = zeros(num_steps, 1);
    new_Theta = zeros(num_steps, 1);
    new_V = zeros(num_steps, 1);
    new_Time = zeros(num_steps, 1);
    
    for i = 1:num_steps
        curr_time = curr_time + dt;
        curr_x = curr_x + curr_v * cos(curr_theta) * dt;
        curr_y = curr_y + curr_v * sin(curr_theta) * dt;
        curr_theta = curr_theta + w * dt;
        curr_v = curr_v + a * dt;
        
        curr_theta = mod(curr_theta + pi, 2*pi) - pi;
        
        new_X(i) = curr_x;
        new_Y(i) = curr_y;
        new_Theta(i) = curr_theta;
        new_V(i) = curr_v;
        new_Time(i) = curr_time;
    end
    
    veh.X_true =[veh.X_true; new_X];
    veh.Y_true = [veh.Y_true; new_Y];
    veh.Theta_true =[veh.Theta_true; new_Theta];
    veh.V_true =[veh.V_true; new_V];
    veh.Time_true = [veh.Time_true; new_Time];
end