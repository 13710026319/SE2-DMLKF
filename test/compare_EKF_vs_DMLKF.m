% =========================================================================
% compare_EKF_vs_DMLKF.m
% 比较 EKF (General) 与 SE(2)-DMLKF 在多车协同定位下的表现
% 功能：
% 1. 同步运行两种滤波器
% 2. 绘制 5 辆车误差对比子图
% 3. 命令行输出各车辆的 RMSE 对比
% =========================================================================
clc; clear; close all;

%% 1. 环境准备
addpath(genpath('Common'));
addpath(genpath('Filters'));

trajectories_mat_name = 'Trj_data_Veh5_Anc2.mat';

data_path = fullfile('DataGenerator', trajectories_mat_name);
if ~exist(data_path, 'file')
    error('未找到数据文件，请先运行 generate_data.m');
end
load(data_path); 

%% 2. 初始化参数与滤波器
num_vehicles = 5;
num_anchor = 2;
v_names = {'V1', 'V2', 'V3', 'V4', 'V5'};
ekf_filters = cell(1, num_vehicles);
dmlkf_filters = cell(1, num_vehicles);

for i = 1:num_vehicles
    veh = trajectories.(v_names{i});
    
    % 初始状态定义
    th = veh.Theta_true(1);
    vx = veh.V_true(1) * cos(th);
    vy = veh.V_true(1) * sin(th);
    px = veh.X_true(1);
    py = veh.Y_true(1);
    
    init_state.state.T = eye(4);
    init_state.state.T(1:2, 1:2) = [cos(th), -sin(th); sin(th), cos(th)];
    init_state.state.T(1:2, 3)   = [vx; vy]; % 速度在第3列
    init_state.state.T(1:2, 4)   = [px; py]; % 位置在第4列
    init_state.state.ba = [0; 0];
    init_state.state.bw = 0;
    init_state.P = eye(8) * 0.1;
    
    init_state.noise_params = IMU_noise_params;
    init_state.noise_params.sigma_anc = UWB_noise_params.sigma_anc;
    init_state.noise_params.sigma_rel = UWB_noise_params.sigma_rel;
    
    % 实例化两种滤波器
    % ekf_filters{i} = EKF_filter(init_state, 'general');
    ekf_filters{i} = IEKF_filter(init_state, 'general');
    dmlkf_filters{i} = DMLKF(init_state);
end

t_imu = trajectories.V1.IMU_Time;
N_steps = length(t_imu);
dt = 0.02;
t_uwb = trajectories.V1.UWB_Anchor(:, 1);
uwb_idx = 1;

% 预分配状态历史 (N_steps x 8维 x 5车 x 2种算法)
history_EKF = zeros(N_steps, 8, num_vehicles);
history_DMLKF = zeros(N_steps, 8, num_vehicles);

fprintf('开始对比仿真：EKF vs DMLKF...\n');

%% 3. 同步仿真循环
for k = 1:N_steps
    curr_t = t_imu(k);
    
    % --- [步骤 A] IMU 预测 (两种算法同步) ---
    for i = 1:num_vehicles
        veh = trajectories.(v_names{i});
        am = veh.IMU_acc_m(k, :)';
        wm = veh.IMU_gyro_m(k);
        
        ekf_filters{i}.predict(am, wm, dt);
        dmlkf_filters{i}.predict(am, wm, dt);
    end
    
    % --- [步骤 B] UWB 更新 ---
    if uwb_idx <= length(t_uwb) && abs(curr_t - t_uwb(uwb_idx)) < 1e-4
        % 1. EKF 逻辑：提取预测位置并更新
        ekf_pred_pos = zeros(num_vehicles, 2);
        for i = 1:num_vehicles
            ekf_pred_pos(i, :) = ekf_filters{i}.state.T(1:2, 4)';
        end
        
        % 2. DMLKF 逻辑：准备广播消息
        dmlkf_msgs = cell(1, num_vehicles);
        for i = 1:num_vehicles
            dmlkf_msgs{i} = dmlkf_filters{i}.broadcast_step();
        end
        
        % 3. 执行更新
        for i = 1:num_vehicles
            veh = trajectories.(v_names{i});
            anc_meas = veh.UWB_Anchor(uwb_idx, 2:(1+num_anchor))'; 
            rel_meas = veh.UWB_Relative(uwb_idx, 2:6)'; 
            
            % EKF 更新
            ekf_filters{i}.update_anchor(anc_meas, anchors);
            ekf_filters{i}.update_general(rel_meas, ekf_pred_pos);
            
            % DMLKF 更新
            dmlkf_filters{i}.update_DMLKF(anc_meas, anchors, rel_meas, dmlkf_msgs);

        end
        uwb_idx = uwb_idx + 1;
    end
    
    % --- [步骤 C] 记录结果 ---
    for i = 1:num_vehicles
        % 记录 EKF
        T_e = ekf_filters{i}.state.T;
        history_EKF(k, :, i) = [T_e(1:2, 4)', atan2(T_e(2,1), T_e(1,1)), ...
                                T_e(1:2, 3)', ekf_filters{i}.state.ba', ekf_filters{i}.state.bw];
        % 记录 DMLKF
        T_d = dmlkf_filters{i}.state.T;
        history_DMLKF(k, :, i) = [T_d(1:2, 4)', atan2(T_d(2,1), T_d(1,1)), ...
                                  T_d(1:2, 3)', dmlkf_filters{i}.state.ba', dmlkf_filters{i}.state.bw];

    end
end

%% 4. 数据处理与 RMSE 计算
fprintf('\n--- 性能指标分析 (RMSE) ---\n');
fprintf('%-10s | %-15s | %-15s | %-10s\n', 'Vehicle', 'EKF RMSE(m)', 'DMLKF RMSE(m)', 'Improvement');
fprintf('-------------------------------------------------------------\n');

res_EKF = cell(1, num_vehicles);
res_DMLKF = cell(1, num_vehicles);

for i = 1:num_vehicles
    res_EKF{i} = process_filter_result(trajectories.(v_names{i}), history_EKF(:,:,i));
    res_DMLKF{i} = process_filter_result(trajectories.(v_names{i}), history_DMLKF(:,:,i));
    
    rmse_e = sqrt(mean(res_EKF{i}.errors.err_Horizontal.^2));
    rmse_d = sqrt(mean(res_DMLKF{i}.errors.err_Horizontal.^2));
    imp = (rmse_e - rmse_d) / rmse_e * 100;
    
    fprintf('%-10s | %-15.4f | %-15.4f | %-9.2f%%\n', v_names{i}, rmse_e, rmse_d, imp);
end


%% 5. 可视化对比图
figure('Name', 'Algorithm Comparison: EKF vs DMLKF', 'Position', [100, 50, 900, 900]);
sgtitle('Horizontal Position Error Comparison: EKF vs SE(2)-DMLKF', 'FontSize', 14, 'FontWeight', 'bold');

for i = 1:num_vehicles
    subplot(5, 1, i);
    t_arr = res_EKF{i}.time;
    
    hold on;
    plot(t_arr, res_EKF{i}.errors.err_Horizontal, 'r--', 'LineWidth', 1.0, 'DisplayName', 'EKF (General)');
    plot(t_arr, res_DMLKF{i}.errors.err_Horizontal, 'b-', 'LineWidth', 1.2, 'DisplayName', 'SE(2)-DMLKF');
    hold off;
    
    grid on;
    ylabel('Error (m)');
    title(['Vehicle ', num2str(i), ' (', v_names{i}, ')']);
    
    % 自适应纵坐标范围
    max_err = max([max(res_EKF{i}.errors.err_Horizontal), max(res_DMLKF{i}.errors.err_Horizontal)]);
    ylim([0, max(0.5, max_err * 1.1)]);
    
    if i == 1
        legend('Location', 'northeast');
    end
    if i == num_vehicles
        xlabel('Time (s)');
    end
end