% =========================================================================
% SE(2)-DMLKF 算法仿真数据生成脚本
% 环境：20m x 40m 无遮挡空间
% 基站：2个，位于对角线 (0,0) 和 (20,40)
% 车辆：5辆，不同起点，运行5分钟(300秒)，车速降低，轨迹简化(<=2次转弯)
% 采样：dt = 0.5s，每车共 600 个轨迹点
% =========================================================================

% _2是两个基站下的实验数据，01、02等分别表示备用的不同数据集

clc; clear; close all;

%% 1. 全局参数设置
dt = 0.5;                % 采样时间 0.5秒
anchor_num = 2;
vehicle_num = 5;

% anchors =[0, 0;          % 基站1位置
%           20,40];       % 基站2位置

anchors =[20, 0;          % 基站1位置
          0,40];       % 基站2位置

% anchors = [10,20];

% anchors = [0,0;
%             10,20;
%             20,40];

% 设置目标文件夹路径
save_dir = 'E:\SE2-MLKF-Project\DataGenerator';
trajectories_mat_name = 'Trj_data_Veh5_Anc2_new.mat';

% 创建总体轨迹结构体
trajectories = struct();

% 创建IMU噪声结构体
IMU_noise_params = struct();

% 创建UWB噪声结构体
UWB_noise_params = struct();

trajectories.V1 = init_vehicle(4,  2,  pi/2, 0.08);  % 左下角，朝北
trajectories.V2 = init_vehicle(16, 5,  pi/2, 0.08);  % 右下角，朝北
trajectories.V3 = init_vehicle(6,  10, 0,    0.08);  % 左侧中下，朝东
trajectories.V4 = init_vehicle(3,  32, 0,    0.08);  % 左侧中上，朝东
trajectories.V5 = init_vehicle(17, 35, pi,   0.08);  % 右上角，朝西


%% 2. 拼接生成每辆车的轨迹 (简化版：每车仅 1 次转弯，包含加减速)
% 采样说明：总持续时间 299.5s，配合初始点共 600 个点。
% 转弯设置：pi/60 rad/s 持续 30s 正好转 90 度。

% ---------------- Vehicle 1: 北 -> 东 (右转) ----------------
% 起点(4, 2) -> 北行 -> 右转 -> 东行
trajectories.V1 = add_trajectory_segment(trajectories.V1,  0.001, 0,      40,   dt); % 加速↑
trajectories.V1 = add_trajectory_segment(trajectories.V1,  0,     0,      100,  dt); % 匀速↑
trajectories.V1 = add_trajectory_segment(trajectories.V1, -0.001, 0,      40,   dt); % 减速↑
trajectories.V1 = add_trajectory_segment(trajectories.V1,  0,    -pi/60,  30,   dt); % 右转90°: N→E
trajectories.V1 = add_trajectory_segment(trajectories.V1,  0.001, 0,      40,   dt); % 加速→
trajectories.V1 = add_trajectory_segment(trajectories.V1, -0.001, 0,      49.5, dt); % 减速→
% 合计: 40+100+40+30+40+49.5 = 299.5s

% ---------------- Vehicle 2: 北 -> 西 (左转) ----------------
% 起点(16, 5) -> 北行 -> 左转 -> 西行
trajectories.V2 = add_trajectory_segment(trajectories.V2,  0.0015,0,      30,   dt); % 加速↑
trajectories.V2 = add_trajectory_segment(trajectories.V2,  0,     0,      110,  dt); % 匀速↑
trajectories.V2 = add_trajectory_segment(trajectories.V2, -0.0015,0,      30,   dt); % 减速↑
trajectories.V2 = add_trajectory_segment(trajectories.V2,  0,     pi/60,  30,   dt); % 左转90°: N→W
trajectories.V2 = add_trajectory_segment(trajectories.V2,  0.001, 0,      50,   dt); % 加速←
trajectories.V2 = add_trajectory_segment(trajectories.V2, -0.001, 0,      49.5, dt); % 减速←
% 合计: 30+110+30+30+50+49.5 = 299.5s

% ---------------- Vehicle 3: 东 -> 北 (左转) ----------------
% 起点(2, 10) -> 东行 -> 左转 -> 北行
trajectories.V3 = add_trajectory_segment(trajectories.V3,  0.001, 0,      10,   dt); % 加速→
trajectories.V3 = add_trajectory_segment(trajectories.V3,  0,     0,      90,   dt); % 匀速→
trajectories.V3 = add_trajectory_segment(trajectories.V3, -0.002, 0,      20,   dt); % 减速→
trajectories.V3 = add_trajectory_segment(trajectories.V3,  0,     pi/60,  30,   dt); % 左转90°: E→N
trajectories.V3 = add_trajectory_segment(trajectories.V3,  0.001, 0,      60,   dt); % 加速↑
trajectories.V3 = add_trajectory_segment(trajectories.V3, -0.001, 0,      89.5, dt); % 减速↑
% 合计: 20+80+20+30+60+89.5 = 299.5s

% ---------------- Vehicle 4: 东 -> 南 (右转) ----------------
% 起点(3, 35) -> 东行 -> 右转 -> 南行
trajectories.V4 = add_trajectory_segment(trajectories.V4,  0.001, 0,      40,   dt); % 加速→
trajectories.V4 = add_trajectory_segment(trajectories.V4,  0,     0,      80,   dt); % 匀速→
trajectories.V4 = add_trajectory_segment(trajectories.V4,  0,    -pi/60,  30,   dt); % 右转90°: E→S
trajectories.V4 = add_trajectory_segment(trajectories.V4, -0.001, 0,      50,   dt); % 减速→
trajectories.V4 = add_trajectory_segment(trajectories.V4,  0.001, 0,      50,   dt); % 加速↓
trajectories.V4 = add_trajectory_segment(trajectories.V4, -0.001, 0,      49.5, dt); % 减速↓
% 合计: 50+70+50+30+50+49.5 = 299.5s

% ---------------- Vehicle 5: 西 -> 南 (左转) ----------------
% 起点(18, 30) -> 西行 -> 左转 -> 南行
trajectories.V5 = add_trajectory_segment(trajectories.V5,  0.001, 0,      30,   dt); % 加速←
trajectories.V5 = add_trajectory_segment(trajectories.V5,  0,     0,      100,   dt); % 匀速←
trajectories.V5 = add_trajectory_segment(trajectories.V5,  0,     pi/60,  30,   dt); % 左转90°: W→S
trajectories.V5 = add_trajectory_segment(trajectories.V5, -0.001, 0,      40,   dt); % 减速←
trajectories.V5 = add_trajectory_segment(trajectories.V5,  0.001, 0,      50,   dt); % 加速↓
trajectories.V5 = add_trajectory_segment(trajectories.V5, -0.001, 0,      49.5, dt); % 减速↓
% 合计: 40+90+40+30+50+49.5 = 299.5s

%% 3 生成50Hz的IMU数据 (带随机游走偏置和高斯噪声)
f_imu = 50;                  % IMU频率 50Hz
dt_imu = 1 / f_imu;          % IMU采样间隔 0.02s

% 模拟典型MEMS IMU噪声参数 (可根据后续滤波效果调整)
IMU_noise_params.sigma_na = 0.05;      % 加速度计高斯白噪声标准差 (m/s^2)
IMU_noise_params.sigma_nw = 0.005;     % 陀螺仪高斯白噪声标准差 (rad/s)
IMU_noise_params.sigma_ba = 0.002;     % 加速度偏置随机游走标准差 (m/s^2 * sqrt(s))
IMU_noise_params.sigma_bw = 0.0002;    % 陀螺仪偏置随机游走标准差 (rad/s * sqrt(s))

v_names = {'V1', 'V2', 'V3', 'V4', 'V5'};
for i = 1:5
    veh = trajectories.(v_names{i});
    t_traj = veh.Time_true;
    
    % 生成50Hz的时间戳
    t_imu = (t_traj(1) : dt_imu : t_traj(end))';
    N_imu = length(t_imu);
    
    % 1. 插值真实的轨迹状态到 50Hz (必须先解包角度避免2pi附近的跳变)
    theta_unwrapped = unwrap(veh.Theta_true);
    theta_imu_unwrapped = interp1(t_traj, theta_unwrapped, t_imu, 'linear');
    theta_imu = mod(theta_imu_unwrapped + pi, 2*pi) - pi;
    v_imu_mag = interp1(t_traj, veh.V_true, t_imu, 'linear');
    
    % 提取全局坐标系下的真实速度 Vx, Vy
    vx_imu = v_imu_mag .* cos(theta_imu);
    vy_imu = v_imu_mag .* sin(theta_imu);
    
    % 2. 计算理想的运动学导数 (使用中心差分求导)
    % 真实角速度
    w_true = gradient(theta_imu_unwrapped, dt_imu);
    % 真实全局坐标系下的加速度
    ax_true = gradient(vx_imu, dt_imu);
    ay_true = gradient(vy_imu, dt_imu);
    
    % 论文 Eq 16: v_dot = R(theta) * (am - ba - na)
    % 真实机体坐标系下的理想加速度 a_b = R(theta)^T * a_world
    ab_x_true = ax_true .* cos(theta_imu) + ay_true .* sin(theta_imu);
    ab_y_true = -ax_true .* sin(theta_imu) + ay_true .* cos(theta_imu);
    
    % 3. 生成随机游走偏置 (Bias Evolution)
    % 设定一个初始的不确定的偏置值
    ba_init = (rand(1, 2) - 0.5) * 0.2;  % 加速度初始偏置 [-0.1, 0.1] m/s^2
    bw_init = (rand(1, 1) - 0.5) * 0.02; % 陀螺仪初始偏置 [-0.01, 0.01] rad/s
    
    % 偏置随时间随机游走 (Discrete Integration of White Noise)
    b_a_x = ba_init(1) + cumsum(randn(N_imu, 1) * IMU_noise_params.sigma_ba * sqrt(dt_imu));
    b_a_y = ba_init(2) + cumsum(randn(N_imu, 1) * IMU_noise_params.sigma_ba * sqrt(dt_imu));
    b_w   = bw_init + cumsum(randn(N_imu, 1) * IMU_noise_params.sigma_bw * sqrt(dt_imu));
    
    % 4. 生成白噪声，叠加偏置和白噪声，生成最终传感器测量值
    acc_noise_x = randn(N_imu, 1) * IMU_noise_params.sigma_na;
    acc_noise_y = randn(N_imu, 1) * IMU_noise_params.sigma_na;
    gyro_noise  = randn(N_imu, 1) * IMU_noise_params.sigma_nw;
    
    acc_m_x = ab_x_true + b_a_x + acc_noise_x;
    acc_m_y = ab_y_true + b_a_y + acc_noise_y;
    gyro_m  = w_true + b_w + gyro_noise;
    
    % 5. 将数据保存到结构体中，同时保存真实的bias用作后续滤波器的评价基准 (Ground Truth)
    trajectories.(v_names{i}).IMU_Time = t_imu;
    trajectories.(v_names{i}).IMU_acc_m =[acc_m_x, acc_m_y]; % N x 2 矩阵 b系
    trajectories.(v_names{i}).IMU_gyro_m = gyro_m;            % N x 1 矩阵
    
    trajectories.(v_names{i}).IMU_bias_a_true =[b_a_x, b_a_y];
    trajectories.(v_names{i}).IMU_bias_w_true = b_w;
end


%% 4. 生成 10Hz 的 UWB 测距数据 (基站测距和相对测距)
f_uwb = 10;                     % UWB频率 10Hz
dt_uwb = 1 / f_uwb;             % UWB采样间隔 0.1s

% 补充 UWB 噪声参数设置 (假设已有 UWB_noise_params = struct();)
UWB_noise_params.sigma_anc = 0.06;  % 基站测距高斯白噪声标准差 (假设误差0.03m)
UWB_noise_params.sigma_rel = 0.06; % 相对测距高斯白噪声标准差 (假设动态误差0.07m)

% 提取公共的最长时间跨度
t_end_all = trajectories.V1.Time_true(end);
t_uwb = (0 : dt_uwb : t_end_all)';
N_uwb = length(t_uwb);

% 预先插值出所有车在 10Hz 下的真实位置，方便互相求相对距离
% pos_true_uwb: N_uwb x 2 x 5
pos_true_uwb = zeros(N_uwb, 2, 5);
v_names = {'V1', 'V2', 'V3', 'V4', 'V5'};
for i = 1:5
    veh = trajectories.(v_names{i});
    pos_true_uwb(:, 1, i) = interp1(veh.Time_true, veh.X_true, t_uwb, 'linear');
    pos_true_uwb(:, 2, i) = interp1(veh.Time_true, veh.Y_true, t_uwb, 'linear');
end

for i = 1:5
    v_name = v_names{i};
    
    % ---------------- A. 基站测距 ----------------
    % 数据集格式:[时间, 基站1测距, 基站2测距]
    UWB_Anchor = zeros(N_uwb, 1 + anchor_num);
    UWB_Anchor(:, 1) = t_uwb;
    
    for a_idx = 1:size(anchors, 1)
        dx = pos_true_uwb(:, 1, i) - anchors(a_idx, 1);
        dy = pos_true_uwb(:, 2, i) - anchors(a_idx, 2);
        dist_true = sqrt(dx.^2 + dy.^2);
        
        % 叠加高斯测距噪声
        UWB_Anchor(:, 1 + a_idx) = dist_true + randn(N_uwb, 1) * UWB_noise_params.sigma_anc;
    end
    trajectories.(v_name).UWB_Anchor = UWB_Anchor;
    
    % ---------------- B. 相对测距 ----------------
    % 数据集格式:[时间, 与V1距离, 与V2距离, 与V3距离, 与V4距离, 与V5距离]
    UWB_Relative = zeros(N_uwb, 1 + vehicle_num);
    UWB_Relative(:, 1) = t_uwb;
    
    for j = 1:vehicle_num
        if i == j
            % 自己到自己的距离设为 NaN，便于后续滤波算法中通过 isnan() 忽略跳过
            UWB_Relative(:, 1 + j) = NaN;
        else
            dx = pos_true_uwb(:, 1, i) - pos_true_uwb(:, 1, j);
            dy = pos_true_uwb(:, 2, i) - pos_true_uwb(:, 2, j);
            dist_true = sqrt(dx.^2 + dy.^2);
            
            % 相对测距增加略大的高斯噪声
            UWB_Relative(:, 1 + j) = dist_true + randn(N_uwb, 1) * UWB_noise_params.sigma_rel;
        end
    end
    trajectories.(v_name).UWB_Relative = UWB_Relative;
end

%% 5. 结果验证与可视化
figure('Name', 'Multi-Agent UWB/IMU Trajectories', 'Position',[100, 100, 500, 700]);
hold on; grid on; axis equal;
% 适应缩小的 20x40m 场地调整坐标轴限制
xlim([-2, 22]); ylim([-2, 42]);
xlabel('X Position (m)'); ylabel('Y Position (m)');
title('5 Vehicles Trajectories (300s, 20x40m Environment)');

% 绘制边界 (对应新的环境尺寸)
rectangle('Position',[0, 0, 20, 40], 'EdgeColor', 'k', 'LineWidth', 1.5, 'LineStyle', '--');

% 绘制基站
h_anchor = plot(anchors(:,1), anchors(:,2), '^', 'MarkerSize', 12, ...
    'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k', 'DisplayName', 'UWB Anchors');

% 绘制每辆小车的轨迹
colors = lines(5);
v_names = {'V1', 'V2', 'V3', 'V4', 'V5'};
h_traj = zeros(1, 5);

for i = 1:5
    v_data = trajectories.(v_names{i});
    
    % 输出验证信息到控制台
    fprintf('%s 轨迹点数: %d, IMU数据点数: %d, 最终位置: (%.1f, %.1f)\n', ...
        v_names{i}, length(v_data.Time_true), length(v_data.IMU_Time), v_data.X_true(end), v_data.Y_true(end));
    
    % 绘制轨迹线 (注意这里改为了 X_true 和 Y_true)
    h_traj(i) = plot(v_data.X_true, v_data.Y_true, 'Color', colors(i,:), 'LineWidth', 2, ...
        'DisplayName', sprintf('Vehicle %d', i));
    
    % 绘制起点和终点标识
    plot(v_data.X_true(1), v_data.Y_true(1), 'o', 'MarkerSize', 6, 'MarkerFaceColor', colors(i,:), 'Color', colors(i,:));
    plot(v_data.X_true(end), v_data.Y_true(end), '*', 'MarkerSize', 8, 'Color', colors(i,:));
end

legend([h_anchor, h_traj], 'Location', 'northeastoutside');
hold off;

%% 6. 保存生成的实验数据到指定目录


% % 如果文件夹不存在，则自动创建
% if ~exist(save_dir, 'dir')
%     mkdir(save_dir);
% end
% 
% % 拼接完整的文件保存路径
% save_path = fullfile(save_dir, trajectories_mat_name);
% 
% % 保存关键变量 (将 轨迹数据、基站位置及 噪声参数 打包保存)
% save(save_path, 'trajectories', 'anchors', 'IMU_noise_params', 'UWB_noise_params');
% 
% fprintf('\n==== 数据生成完成 ====\n');
% fprintf('UWB 数据已按 10Hz 生成，基站/相对测距维度配置正确。\n');
% fprintf('所有数据已成功保存至: %s\n', save_path);


%% =========================================================================
% 内部函数定义区域
% =========================================================================

function veh = init_vehicle(x0, y0, theta0, v0)
    % 仅设置初始状态 (t = 0 时刻)，加入 _true 后缀
    veh.X_true = x0;
    veh.Y_true = y0;
    veh.Theta_true = theta0;
    veh.V_true = v0;
    veh.Time_true = 0;
end

% 2. 轨迹段生成函数
function veh = add_trajectory_segment(veh, a, w, duration, dt)
    % veh: 当前车辆的轨迹结构体
    % a: 线性加速度 (m/s^2)
    % w: 角速度 (rad/s)
    % duration: 该段运动持续时间 (s)
    % dt: 采样间隔 (s)

    num_steps = round(duration / dt);
    
    % 获取当前末端真实状态
    curr_x = veh.X_true(end);
    curr_y = veh.Y_true(end);
    curr_theta = veh.Theta_true(end);
    curr_v = veh.V_true(end);
    curr_time = veh.Time_true(end);
    
    % 预分配内存
    new_X = zeros(num_steps, 1);
    new_Y = zeros(num_steps, 1);
    new_Theta = zeros(num_steps, 1);
    new_V = zeros(num_steps, 1);
    new_Time = zeros(num_steps, 1);
    
    for i = 1:num_steps
        % 更新状态 (理想运动学模型)
        curr_time = curr_time + dt;
        curr_x = curr_x + curr_v * cos(curr_theta) * dt;
        curr_y = curr_y + curr_v * sin(curr_theta) * dt;
        curr_theta = curr_theta + w * dt;
        curr_v = curr_v + a * dt;
        
        % 将角度约束在 [-pi, pi] 之间
        curr_theta = mod(curr_theta + pi, 2*pi) - pi;
        
        % 记录数据
        new_X(i) = curr_x;
        new_Y(i) = curr_y;
        new_Theta(i) = curr_theta;
        new_V(i) = curr_v;
        new_Time(i) = curr_time;
    end
    
    % 将新段拼接回原结构体
    veh.X_true =[veh.X_true; new_X];
    veh.Y_true = [veh.Y_true; new_Y];
    veh.Theta_true =[veh.Theta_true; new_Theta];
    veh.V_true =[veh.V_true; new_V];
    veh.Time_true = [veh.Time_true; new_Time];
end