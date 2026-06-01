function packed_result = process_filter_result(veh_true_data, est_state_history)
    % process_filter_result 整理单个车辆的滤波结果与真值
    % 输入:
    %   veh_true_data     - 数据集中提取的当前车结构体
    %   est_state_history - N x 8 矩阵, 滤波输出历史记录 
    %                       [X, Y, Theta, Vx, Vy, ba_x, ba_y, bw]
    % 输出:
    %   packed_result     - 包含对齐好的数据与误差信息的结构体

    % 1. 将 2Hz 的真实轨迹插值到 50Hz (IMU时间戳)
    t_true = veh_true_data.Time_true;
    t_imu = veh_true_data.IMU_Time;
    
    true_pos_interp = zeros(length(t_imu), 2);
    true_pos_interp(:,1) = interp1(t_true, veh_true_data.X_true, t_imu, 'linear');
    true_pos_interp(:,2) = interp1(t_true, veh_true_data.Y_true, t_imu, 'linear');
    
    theta_unwrapped = unwrap(veh_true_data.Theta_true);
    theta_interp_unwrapped = interp1(t_true, theta_unwrapped, t_imu, 'linear');
    true_theta_interp = mod(theta_interp_unwrapped + pi, 2*pi) - pi;
    true_V_interp = interp1(t_true, veh_true_data.V_true, t_imu, 'linear');
    
    % 2. 提取滤波估计位置，调用分析函数
    est_pos = est_state_history(:, 1:2);
    err_data = analyze_error(est_pos, true_pos_interp, 0); 
    
    % 3. 打包数据
    packed_result.time = t_imu;
    
    packed_result.true_state.X = true_pos_interp(:, 1);
    packed_result.true_state.Y = true_pos_interp(:, 2);
    packed_result.true_state.Theta = true_theta_interp;
    packed_result.true_state.V = true_V_interp;
    
    packed_result.measurements.IMU_acc = veh_true_data.IMU_acc_m;
    packed_result.measurements.IMU_gyro = veh_true_data.IMU_gyro_m;
    packed_result.measurements.UWB_Anchor = veh_true_data.UWB_Anchor;
    packed_result.measurements.UWB_Relative = veh_true_data.UWB_Relative;
    
    packed_result.est_state.X = est_state_history(:, 1);
    packed_result.est_state.Y = est_state_history(:, 2);
    packed_result.est_state.Theta = est_state_history(:, 3);
    packed_result.est_state.Vx = est_state_history(:, 4);
    packed_result.est_state.Vy = est_state_history(:, 5);
    packed_result.est_state.ba_x = est_state_history(:, 6);
    packed_result.est_state.ba_y = est_state_history(:, 7);
    packed_result.est_state.bw = est_state_history(:, 8);
    
    packed_result.errors = err_data;
end