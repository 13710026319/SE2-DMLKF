classdef IEKF_filter < EKF_filter
    % IEKF_filter: 基于流形误差空间的迭代扩展卡尔曼滤波器 (Iterated EKF)
    % 继承自 EKF_filter，保留标准的 IMU 预测和 Anchor 更新，
    % 但重写相对测距更新，引入 Gauss-Newton 迭代以克服高非线性误差。
    
    methods
        function obj = IEKF_filter(Init_state, flag, use_ci)
            % 构造函数
            if nargin < 3, use_ci = false; end
            if nargin < 2, flag = 'general'; end % 默认开启 general 模式
            if nargin < 1, Init_state = []; end
            
            % 调用父类构造函数
            obj@EKF_filter(Init_state, flag, use_ci);
        end
        
        function update_general(obj, rel_meas, neighbor_pos)
            % 迭代相对测距更新 (Iterated EKF Update)
            
            if strcmp(obj.flag, 'anchor_only')
                return; % 'anchor_only' 模式下跳过相对更新
            end
            
            % 1. 过滤无效测量
            valid_idx = find(~isnan(rel_meas));
            num_valid = length(valid_idx);
            if num_valid == 0, return; end
            
            meas_valid = rel_meas(valid_idx);
            pos_valid  = neighbor_pos(valid_idx, :);
            
            % 2. 配置观测噪声与 CI 乱伦防御
            R_rel = eye(num_valid) * (obj.noise_params.sigma_rel^2);
            if obj.use_ci
                w_m = 1.0 / num_valid;
                R_rel = R_rel / w_m; % CI 膨胀观测噪声
            end
            
            % 3. IEKF 迭代准备
            state_prior = obj.state;
            P_prior = obj.P;
            
            % 初始化工作点 (Operating Point)
            state_op = state_prior; 
            dx_s = zeros(8, 1); % 记录当前工作点相对于先验的累积误差状态
            
            max_s = 20;
            eta_NR = 1e-4;
            
            H_rel = zeros(num_valid, 8);
            r_rel = zeros(num_valid, 1);
            
            % 4. 高斯-牛顿迭代 (Gauss-Newton Optimization on Manifold)
            for s = 0 : max_s-1
                % 提取当前工作点的状态
                p_op = state_op.T(1:2, 4);
                R_op = state_op.T(1:2, 1:2);
                
                % 工作点处的降维算子 (SE_2(2) 对应的 2x8 算子)
                S_p_op = [zeros(2, 3), R_op, zeros(2, 3)];
                
                % 计算残差和工作点处的雅可比
                for i = 1:num_valid
                    n_i = pos_valid(i, :)';
                    delta_p = p_op - n_i;
                    
                    % 防除 0 保护
                    dist_op = max(norm(delta_p), 1e-6); 
                    
                    r_rel(i) = meas_valid(i) - dist_op;
                    
                    c_i_T = delta_p' / dist_op;
                    H_rel(i, :) = c_i_T * S_p_op;
                end
                
                % 计算卡尔曼增益 K (基于工作点处的 H)
                S = H_rel * P_prior * H_rel' + R_rel;
                K = P_prior * H_rel' / S;
                
                % 【核心理论】: IEKF 的误差状态更新公式
                % 等效残差补偿了当前工作点相对于先验已经偏离的部分
                dx_new = K * (r_rel + H_rel * dx_s);
                
                % 检查收敛 (状态变化量极小)
                if norm(dx_new - dx_s) < eta_NR
                    dx_s = dx_new;
                    state_op = boxplus_Ms(state_prior, dx_s);
                    break;
                end
                
                % 步进更新
                dx_s = dx_new;
                state_op = boxplus_Ms(state_prior, dx_s);
            end
            
            % 5. 迭代收敛后，正式应用更新
            obj.state = state_op;
            
            % 最终协方差更新 (必须使用收敛后的最后一次 H 和 K，采用稳定 Joseph 形式)
            I_KH = eye(8) - K * H_rel;
            obj.P = I_KH * P_prior * I_KH' + K * R_rel * K';
            obj.P = (obj.P + obj.P') / 2; % 保证对称正定
        end
    end
end


