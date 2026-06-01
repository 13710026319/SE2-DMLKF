% =========================================================================
% test_EKF_compare.m
% 对比测试: 纯基站 EKF (Anchor-Only) vs 基站+相对测距 EKF (General)
% 目的: 验证引入相对测距(协同定位)对降低定位误差的具体贡献
% 适配 SE_2(2) 流形 (4x4 变换矩阵)
% =========================================================================
clc; clear; close all;

%% 1. 确保路径已经添加
addpath(genpath('Common'));
addpath(genpath('Filters'));

%% 2. 加载数据
data_path = fullfile('DataGenerator', 'measurement_data_2.mat');
if ~exist(data_path, 'file')
    error('未找到数据文件，请先运行 DataGenerator/generate_data.m');
end
load(data_path); % 载入 trajectories, anchors, IMU_noise_params, UWB_noise_params

%% 3. 实例化两组滤波器
num_vehicles = 5;
num_anchor = 2;
filters_AnchorOnly = cell(1, num_vehicles); % 仅基站更新
filters_General    = cell(1, num_vehicles); % 基站+相对测距组

v_names = {'V1', 'V2', 'V3', 'V4', 'V5'};

for i = 1:num_vehicles
    veh = trajectories.(v_names{i});
    
    % 【修改点】：全新的 4x4 SE_2(2) 初始状态组装
    th = veh.Theta_true(1);
    vx = veh.V_true(1) * cos(th);
    vy = veh.V_true(1) * sin(th);
    px = veh.X_true(1);
    py = veh.Y_true(1);
    
    init_state.state.T = eye(4);
    init_state.state.T(1:2, 1:2) = [cos(th), -sin(th); sin(th), cos(th)];
    init_state.state.T(1:2, 3)   = [vx; vy]; % 速度放第3列
    init_state.state.T(1:2, 4)   = [px; py]; % 位置放第4列
    
    init_state.state.ba = [0; 0];
    init_state.state.bw = 0;
    init_state.P = eye(8) * 0.1;
    
    init_state.noise_params = IMU_noise_params;
    init_state.noise_params.sigma_anc = UWB_noise_params.sigma_anc;
    init_state.noise_params.sigma_rel = UWB_noise_params.sigma_rel;
    
    % 分别以不同 flag 实例化 (默认不开启 CI，便于展示朴素 General 带来的漂移)
    filters_AnchorOnly{i} = EKF_filter(init_state, 'anchor_only', false);
    filters_General{i}    = EKF_filter(init_state, 'general', false);
end

% 获取循环步数
t_imu = trajectories.V1.IMU_Time;
N_steps = length(t_imu);
dt = 0.02; % IMU 50Hz

% 预分配位置历史数组：N_steps x 2维(X,Y) x 5辆车
est_pos_AO  = zeros(N_steps, 2, num_vehicles); % AO = Anchor Only
est_pos_Gen = zeros(N_steps, 2, num_vehicles); % Gen = General

t_uwb = trajectories.V1.UWB_Anchor(:, 1);
uwb_idx = 1;

fprintf('开始执行对比仿真测试 (Anchor-Only vs General EKF)...\n');

%% 4. 仿真主循环
for k = 1:N_steps
    curr_t = t_imu(k);
    
    % [步骤 A] IMU 预测
    for i = 1:num_vehicles
        veh = trajectories.(v_names{i});
        am = veh.IMU_acc_m(k, :)';
        wm = veh.IMU_gyro_m(k);
        
        filters_AnchorOnly{i}.predict(am, wm, dt);
        filters_General{i}.predict(am, wm, dt);
    end
    
    % [步骤 B] UWB 更新
    if uwb_idx <= length(t_uwb) && abs(curr_t - t_uwb(uwb_idx)) < 1e-4
        
        % 提取 General 组所有车辆的预测位置 (作为邻居基准)
        pred_pos_Gen = zeros(num_vehicles, 2);
        for i = 1:num_vehicles
            % 【修改点】：位置提取从 T(1:2, 3) 变为 T(1:2, 4)
            pred_pos_Gen(i, :) = filters_General{i}.state.T(1:2, 4)';
        end
        
        for i = 1:num_vehicles
            veh = trajectories.(v_names{i});
            anc_meas = veh.UWB_Anchor(uwb_idx, 2:(1+num_anchor))'; 
            rel_meas = veh.UWB_Relative(uwb_idx, 2:6)'; 
            
            % 1. 仅基站组：只执行 Anchor 更新
            filters_AnchorOnly{i}.update_anchor(anc_meas, anchors);
            
            % 2. General 组：执行 Anchor 更新 + 相对测距更新
            filters_General{i}.update_anchor(anc_meas, anchors);
            filters_General{i}.update_general(rel_meas, pred_pos_Gen);
        end
        uwb_idx = uwb_idx + 1;
    end
    
    % [步骤 C] 记录位置
    for i = 1:num_vehicles
        % 【修改点】：位置提取从 T(1:2, 3) 变为 T(1:2, 4)
        est_pos_AO(k, :, i)  = filters_AnchorOnly{i}.state.T(1:2, 4)';
        est_pos_Gen(k, :, i) = filters_General{i}.state.T(1:2, 4)';
    end
end

fprintf('仿真完成！正在计算 RMSE 并绘制结果...\n');

%% 5. 计算 RMSE 与可视化
figure('Name', 'RMSE Comparison', 'Position',[100, 50, 900, 900]);
sgtitle('Position Error: Anchor-Only vs. General EKF', 'FontSize', 14, 'FontWeight', 'bold');

rmse_AO  = zeros(num_vehicles, 1);
rmse_Gen = zeros(num_vehicles, 1);

for i = 1:num_vehicles
    veh_name = v_names{i};
    veh_data = trajectories.(veh_name);
    
    % 获取 50Hz 真实轨迹
    t_true = veh_data.Time_true;
    true_x_interp = interp1(t_true, veh_data.X_true, t_imu, 'linear');
    true_y_interp = interp1(t_true, veh_data.Y_true, t_imu, 'linear');
    true_pos =[true_x_interp, true_y_interp];
    
    % 计算每一时刻的水平误差
    err_AO  = sqrt(sum((est_pos_AO(:, :, i)  - true_pos).^2, 2));
    err_Gen = sqrt(sum((est_pos_Gen(:, :, i) - true_pos).^2, 2));
    
    % 计算整个轨迹的 RMSE
    rmse_AO(i)  = sqrt(mean(err_AO.^2));
    rmse_Gen(i) = sqrt(mean(err_Gen.^2));
    
    % 绘制子图
    subplot(5, 1, i);
    plot(t_imu, err_AO,  'r-', 'LineWidth', 1.2, 'DisplayName', 'Anchor-Only');
    hold on; grid on;
    plot(t_imu, err_Gen, 'b-', 'LineWidth', 1.2, 'DisplayName', 'Anchor + Relative');
    
    ylabel('Error (m)');
    title(sprintf('Vehicle %d (%s) - RMSE: AO=%.3fm, Gen=%.3fm', i, veh_name, rmse_AO(i), rmse_Gen(i)));
    
    % 统一Y轴上限以便直观对比
    ylim([0, max(max(err_AO), max(err_Gen)) * 1.1]);
    
    if i == 1
        legend('Location', 'northeast');
    end
    if i == num_vehicles
        xlabel('Time (s)');
    end
end

% 控制台输出总结报告
fprintf('\n================== RMSE 对比总结 ==================\n');
fprintf('车辆\t仅基站(AO)\t基站+协同(Gen)\t误差降低幅度\n');
for i = 1:num_vehicles
    improvement = (rmse_AO(i) - rmse_Gen(i)) / rmse_AO(i) * 100;
    fprintf('V%d\t%.3f m\t\t%.3f m\t\t%+.1f %%\n', i, rmse_AO(i), rmse_Gen(i), improvement);
end
fprintf('===================================================\n');
