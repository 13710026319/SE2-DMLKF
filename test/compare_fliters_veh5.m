% =========================================================================
% compare_filters.m
% 比较 EKF,PF,UKF,SE(2)-DMLKF 在多车协同定位下的表现
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

save_result_dir = 'E:\SE2_DMLKF_Project\Result\Veh5';
if ~exist(save_result_dir, 'dir')
    mkdir(save_result_dir);
end

result_full_path = fullfile(save_result_dir, 'Filter_Results_Veh5.mat');

if_read_result = 1; % 1 读取已有结果作图，0 重新生成新的结果保存，注意修改变量名

if if_read_result
    % 功能 1：检查并读取数据
    if exist(result_full_path, 'file')
        fprintf('检测到已有结果，正在加载数据: %s\n', result_full_path);
        load(result_full_path);
        
        % 将结构体中的数据解包回当前工作空间变量名，以匹配后续绘图代码
        res_EKF   = filter_results_Veh5.res_EKF;
        res_PF    = filter_results_Veh5.res_PF;
        res_UKF   = filter_results_Veh5.res_UKF;
        res_DMLKF = filter_results_Veh5.res_DMLKF;
        res_IEKF  = filter_results_Veh5.res_IEKF;

        % 获取车辆数量，用于后续循环
        num_vehicles = length(res_EKF);
        v_names = cell(1, num_vehicles);
        for i = 1:num_vehicles
            v_names{i} = sprintf('V%d', i);
        end
    else
        % 功能 2：如果找不到文件，打印提示并报错终止
        fprintf('【错误】未在 %s 下找到结果文件！\n', save_result_dir);
        return; 
    end
end


if ~if_read_result
    %%  初始化参数与滤波器
    
    num_vehicles = 5;
    num_anchor = 2;
    v_names = {'V1', 'V2', 'V3', 'V4', 'V5'};
    ekf_filters = cell(1, num_vehicles);
    pf_filters = cell(1, num_vehicles);
    ukf_filters = cell(1, num_vehicles);
    dmlkf_filters = cell(1, num_vehicles);
    iekf_filters = cell(1, num_vehicles);

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
        ekf_filters{i} = EKF_filter(init_state, 'general', 0);
        pf_filters{i} = PF_filter(init_state, 'general', 1, 500);
        ukf_filters{i} = UKF_filter(init_state, 'general', 0);
        dmlkf_filters{i} = DMLKF(init_state);
        iekf_filters{i} = IEKF_filter(init_state, 'general', 0);
    end
    
    t_imu = trajectories.V1.IMU_Time;
    N_steps = length(t_imu);
    dt = 0.02;
    t_uwb = trajectories.V1.UWB_Anchor(:, 1);
    uwb_idx = 1;
    
    % 预分配状态历史 (N_steps x 8维 x 5车)
    history_EKF = zeros(N_steps, 8, num_vehicles);
    history_PF = zeros(N_steps, 8, num_vehicles);
    history_UKF = zeros(N_steps, 8, num_vehicles);
    history_DMLKF = zeros(N_steps, 8, num_vehicles);
    history_IEKF = zeros(N_steps, 8, num_vehicles);

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
            pf_filters{i}.predict(am, wm, dt);
            ukf_filters{i}.predict(am, wm, dt);
            dmlkf_filters{i}.predict(am, wm, dt);
            iekf_filters{i}.predict(am, wm, dt);
        end
        
        % --- [步骤 B] UWB 更新 ---
        if uwb_idx <= length(t_uwb) && abs(curr_t - t_uwb(uwb_idx)) < 1e-4
            % 1. EKF 逻辑：提取预测位置并更新
            ekf_pred_pos = zeros(num_vehicles, 2);
            pf_pred_pos = zeros(num_vehicles, 2);
            ukf_pred_pos = zeros(num_vehicles, 2);
            iekf_pred_pos = zeros(num_vehicles, 2);

            for i = 1:num_vehicles
                ekf_pred_pos(i, :) = ekf_filters{i}.state.T(1:2, 4)';
                pf_pred_pos(i, :) = pf_filters{i}.state.T(1:2, 4)';
                ukf_pred_pos(i, :) = ukf_filters{i}.state.T(1:2, 4)';
                iekf_pred_pos(i, :) = iekf_filters{i}.state.T(1:2, 4)';
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
                rel_meas = veh.UWB_Relative(uwb_idx, 2:(1+num_vehicles))'; 
                
                % EKF 更新
                ekf_filters{i}.update_anchor(anc_meas, anchors);
                ekf_filters{i}.update_general(rel_meas, ekf_pred_pos);
                
                % PF 更新
                pf_filters{i}.update_anchor(anc_meas, anchors);
                pf_filters{i}.update_general(rel_meas, pf_pred_pos);
    
                % UKF 更新
                ukf_filters{i}.update_anchor(anc_meas, anchors);
                ukf_filters{i}.update_general(rel_meas, ukf_pred_pos);
    
                % DMLKF 更新
                dmlkf_filters{i}.update_DMLKF(anc_meas, anchors, rel_meas, dmlkf_msgs);

                % IEKF 更新
                iekf_filters{i}.update_anchor(anc_meas, anchors);
                iekf_filters{i}.update_general(rel_meas, iekf_pred_pos);
    
            end
            uwb_idx = uwb_idx + 1;
        end
        
        % --- [步骤 C] 记录结果 ---
        for i = 1:num_vehicles
            % 记录 EKF
            T_ekf = ekf_filters{i}.state.T;
            history_EKF(k, :, i) = [T_ekf(1:2, 4)', atan2(T_ekf(2,1), T_ekf(1,1)), ...
                                    T_ekf(1:2, 3)', ekf_filters{i}.state.ba', ekf_filters{i}.state.bw];
    
            % 记录 PF
            T_pf = pf_filters{i}.state.T;
            history_PF(k, :, i) = [T_pf(1:2, 4)', atan2(T_pf(2,1), T_pf(1,1)), ...
                                    T_pf(1:2, 3)', pf_filters{i}.state.ba', pf_filters{i}.state.bw];
    
            % 记录 UKF
            T_ukf = ukf_filters{i}.state.T;
            history_UKF(k, :, i) = [T_ukf(1:2, 4)', atan2(T_ukf(2,1), T_ukf(1,1)), ...
                                    T_ukf(1:2, 3)', ukf_filters{i}.state.ba', ukf_filters{i}.state.bw];
            % 记录 DMLKF
            T_dmlkf = dmlkf_filters{i}.state.T;
            history_DMLKF(k, :, i) = [T_dmlkf(1:2, 4)', atan2(T_dmlkf(2,1), T_dmlkf(1,1)), ...
                                      T_dmlkf(1:2, 3)', dmlkf_filters{i}.state.ba', dmlkf_filters{i}.state.bw];

            % 记录 IEKF
            T_iekf = iekf_filters{i}.state.T;
            history_IEKF(k, :, i) = [T_iekf(1:2, 4)', atan2(T_iekf(2,1), T_iekf(1,1)), ...
                                    T_iekf(1:2, 3)', iekf_filters{i}.state.ba', iekf_filters{i}.state.bw];
    
        end
    end
    
    %% 4. 数据处理与 RMSE 计算
    
    res_EKF = cell(1, num_vehicles);
    res_PF = cell(1, num_vehicles);
    res_UKF = cell(1, num_vehicles);
    res_DMLKF = cell(1, num_vehicles);
    res_IEKF = cell(1, num_vehicles);

    for i = 1:num_vehicles
        res_EKF{i} = process_filter_result(trajectories.(v_names{i}), history_EKF(:,:,i));
        res_PF{i} = process_filter_result(trajectories.(v_names{i}), history_PF(:,:,i));
        res_UKF{i} = process_filter_result(trajectories.(v_names{i}), history_UKF(:,:,i));
        res_DMLKF{i} = process_filter_result(trajectories.(v_names{i}), history_DMLKF(:,:,i));
        res_IEKF{i} = process_filter_result(trajectories.(v_names{i}), history_IEKF(:,:,i));
    end

    fprintf('\n正在保存数据至 %s ...\n', result_full_path);
    
    % 将 8 辆车的分离数据打包成易于后续调用的整体 cell 结构
    filter_results_Veh5.res_EKF   = res_EKF;
    filter_results_Veh5.res_PF    = res_PF;
    filter_results_Veh5.res_UKF   = res_UKF;
    filter_results_Veh5.res_DMLKF = res_DMLKF;
    filter_results_Veh5.res_IEKF   = res_IEKF;

    % 执行保存命令
    save(result_full_path, 'filter_results_Veh5');
    
    fprintf('==== 结果保存完成！====\n');

end

%% 数据处理与 RMSE 计算
fprintf('\n--- 性能指标分析 (RMSE) ---\n');
fprintf('%-10s | %-15s | %-15s |%-15s |%-15s |%-15s\n', 'Vehicle', 'EKF','PF','UKF', 'IEKF','DMLKF');
fprintf('------------------------------------------------------------------------------------------\n');

for i = 1:num_vehicles
    
    rmse_ekf = sqrt(mean(res_EKF{i}.errors.err_Horizontal.^2));
    rmse_pf = sqrt(mean(res_PF{i}.errors.err_Horizontal.^2));
    rmse_ukf = sqrt(mean(res_UKF{i}.errors.err_Horizontal.^2));
    rmse_dmlkf = sqrt(mean(res_DMLKF{i}.errors.err_Horizontal.^2));
    rmse_iekf = sqrt(mean(res_IEKF{i}.errors.err_Horizontal.^2));

    fprintf('%-10s | %-15.4f | %-15.4f |%-15.4f |%-15.4f |%-15.4f\n', v_names{i}, rmse_ekf, rmse_pf, rmse_ukf, rmse_iekf,rmse_dmlkf);
end

%% 5. 可视化对比图
figure('Name', 'Algorithm Comparison', 'Position', [100, 50, 900, 900]);
sgtitle('Horizontal Position Error Comparison', 'FontSize', 14, 'FontWeight', 'bold');

for i = 1:num_vehicles
    subplot(5, 1, i);
    t_arr = res_EKF{i}.time;
    
    hold on;
    plot(t_arr, res_EKF{i}.errors.err_Horizontal, 'r--', 'LineWidth', 1.0, 'DisplayName', 'EKF (General)');
    plot(t_arr, res_PF{i}.errors.err_Horizontal, 'g--', 'LineWidth', 1.0, 'DisplayName', 'PF (General)');
    plot(t_arr, res_UKF{i}.errors.err_Horizontal, 'y--', 'LineWidth', 1.0, 'DisplayName', 'UKF (General)');
    plot(t_arr, res_DMLKF{i}.errors.err_Horizontal, 'b-', 'LineWidth', 1.2, 'DisplayName', 'SE(2)-DMLKF');
    plot(t_arr, res_IEKF{i}.errors.err_Horizontal, 'c--', 'LineWidth', 1.0, 'DisplayName', 'IEKF (General)');
    hold off;
    
    grid on;
    ylabel('Error (m)');
    title(['Vehicle ', num2str(i), ' (', v_names{i}, ')']);
    
    % 自适应纵坐标范围
    max_err=max([...
    max(res_EKF{i}.errors.err_Horizontal),...
    max(res_IEKF{i}.errors.err_Horizontal),...
    max(res_PF{i}.errors.err_Horizontal),...
    max(res_UKF{i}.errors.err_Horizontal),...
    max(res_DMLKF{i}.errors.err_Horizontal)]);
    ylim([0, max(0.5, max_err * 1.1)]);
    
    if i == 1
        legend('Location', 'northeast');
    end
    if i == num_vehicles
        xlabel('Time (s)');
    end
end