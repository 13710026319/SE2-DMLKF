classdef DMLKF < EKF_filter
    % 目前最优版本，将基站和相对测距一起分到CI权重，当前发现基站93%，相对测距7%
    % 已完美升级至 SE_2(2) 流形 (4x4 变换矩阵)
    methods
        function obj = DMLKF(Init_state)
            % 
            if nargin < 1
                Init_state =[];
            end
            obj@EKF_filter(Init_state, 'anchor_only', false);
        end
        
        function msg = broadcast_step(obj)
            % 论文 5.1 Step 1: Broadcast
            % 输出当前时间 k|k-1 的预测位置 mean 和 位置 covariance
            
            % 【修改点】：在 SE_2(2) 中，位置位于第 4 列
            msg.pos = obj.state.T(1 : 2, 4);
            
            % 【修改点】：Eq 37 全新的降维提取算子 (对应 8D 误差状态)
            R_hat = obj.state.T(1 : 2, 1 : 2);
            S_p = [zeros(2, 3), R_hat, zeros(2, 3)]; % 2x8 算子
            
            % Eq 47: 提取位置的 2x2 协方差
            cov_temp = S_p * obj.P * S_p';
            
            % 【防御性】：强制对称正定，防止网络传输造成的不对称，导致邻居解算NaN
            msg.cov = (cov_temp + cov_temp') / 2;
        end
        

        function update_DMLKF(obj, anc_meas, anc_pos, rel_meas, neighbors_msgs)
            % obj必须只经过了IMU预测而未使用基站更新
            
            % 预备: 保存先验状态 (k|k-1) 与 提取 S_p 算子
            state_prior = obj.state;
            P_prior = obj.P;
            
            % 【防御性】：确保协方差绝对对称
            P_prior = (P_prior + P_prior') / 2;
            
            p_prior = state_prior.T(1 : 2, 4);
            R_hat = state_prior.T(1 : 2, 1 : 2);
            S_p = [zeros(2, 3), R_hat, zeros(2, 3)]; % 2x8 矩阵 (Eq 37)
            
            % =============================================================
            % Step 2: Anchor Range Update via EKF (论文 5.2)
            % =============================================================
            m_anc = length(anc_meas);
            if m_anc > 0
                H_anc = zeros(m_anc, 8);
                r_anc = zeros(m_anc, 1);
                for i = 1 : m_anc
                    a_i = anc_pos(i, :)';
                    delta_p = p_prior - a_i;
                    
                    % 【防御性】：防刚好在基站正上方导致距离0，引发除0出现NaN
                    dist_hat = max(norm(delta_p), 1e-6); 
                    r_anc(i) = anc_meas(i) - dist_hat;
                    c_i_T = delta_p' / dist_hat;
                    H_anc(i, :) = c_i_T * S_p; % Eq 39
                end
                % Joseph 形式更新 Anchor EKF
                R_anc = eye(m_anc) * (obj.noise_params.sigma_anc ^ 2);
                S = H_anc * P_prior * H_anc' + R_anc;
                
                % 【防御性】：通常S较小且正定，使用反斜杠或线性解算比直接乘逆稳
                K = P_prior * H_anc' / S; 
                delta_x_anc = K * r_anc;
                
                I_KH = eye(8) - K * H_anc;
                P_anc = I_KH * P_prior * I_KH' + K * R_anc * K';
                P_anc = (P_anc + P_anc') / 2; % 保证正定对称
            else
                delta_x_anc = zeros(8, 1);
                P_anc = P_prior;
            end
            
            % 转换到信息形式
            % 【防御性】：采用矩阵左除 \ eye(8) 相比 inv(P) 更不易出现极值 NaN
            Omega_anc = P_anc \ eye(8); 
            Omega_anc = (Omega_anc + Omega_anc') / 2;
            eta_anc = Omega_anc * delta_x_anc;
            
            state_anc = boxplus_Ms(state_prior, delta_x_anc);
            p_anc = state_anc.T(1 : 2, 4); % 【修改点】：位置位于第 4 列
            
            % =============================================================
            % Step 3: Relative-Range Processing via MLKF (论文 5.3)
            % =============================================================
            U_list = {};
            sigma_ind2_list = {};
            sigma_dep2_list = {};
            x_ML_list = {};
            
            num_neighbors = length(rel_meas);
            sigma_m2 = obj.noise_params.sigma_rel ^ 2;
            valid_rel_count = 0; % 统计有效的邻居数量
            
            for m = 1 : num_neighbors
                if isnan(rel_meas(m)) || isempty(neighbors_msgs{m})
                    continue;
                end
                
                % 【防御性】：拦截邻居发来的脏数据 (若其发来的位姿已发散崩溃)
                if any(isnan(neighbors_msgs{m}.pos)) || any(isnan(neighbors_msgs{m}.cov(:)))
                    continue;
                end
                
                e_nm = rel_meas(m);
                p_m = neighbors_msgs{m}.pos;
                Sigma_m = neighbors_msgs{m}.cov;
                
                delta_p = p_anc - p_m;
                dist_p = max(norm(delta_p), 1e-6);
                ell_m = delta_p / dist_p;
                
                % 核心理论：拆分独立方差(UWB物理噪声)与相关方差(邻居位置不确定性)
                sigma_ind2 = sigma_m2;
                sigma_dep2 = ell_m' * Sigma_m * ell_m;
                
                % 【防御性】：避免因数值截断导致协方差投影变成负数或绝对0
                sigma_dep2 = max(sigma_dep2, 1e-12); 
                
                x_s = p_anc;
                s = 0; max_s = 20; eta_NR = 1e-4;
                while s < max_s
                    d_s = max(norm(x_s - p_m), 1e-6);
                    u_s = (x_s - p_m) / d_s;
                    r_s = e_nm - d_s;
                    if abs(r_s) < eta_NR
                        break;
                    end
                    x_s = x_s + r_s * u_s;
                    s = s + 1;
                end
                
                x_ML = x_s;
                u_ML = (x_ML - p_m) / max(norm(x_ML - p_m), 1e-6);
                U_nm = u_ML * u_ML'; 
                
                valid_rel_count = valid_rel_count + 1;
                U_list{valid_rel_count} = U_nm;
                sigma_ind2_list{valid_rel_count} = sigma_ind2;
                sigma_dep2_list{valid_rel_count} = sigma_dep2;
                x_ML_list{valid_rel_count} = x_ML;
            end
            
            % =============================================================
            % Step 4: SCI-based Equivalent Unscaled Prior Fusion 
            % (隐式局部SCI：保护IMU状态不被惩罚)
            % =============================================================
            Omega_fused = Omega_anc;
            eta_fused = eta_anc;
            
            if valid_rel_count > 0
                % 【核心理论参数】：显式定义先验的权重(代表基站置信度)
                w_prior = 0.93; 
                
                % 防御：严格保证在 (0, 1) 区间内，否则接下来会除 0 报 NaN
                w_prior = max(min(w_prior, 0.999), 0.001); 
                
                % 剩余权重给相对测距平分
                w_rel_total = 1.0 - w_prior;
                w_m = w_rel_total / valid_rel_count;
                
                % 关键转化：求出等效相对权重比例
                w_equivalent = w_m / w_prior; 
                
                for m = 1 : valid_rel_count
                    x_ML = x_ML_list{m};
                    
                    % 理论实现：仅对相对测距中的 相关方差(邻居方差) 除以极其苛刻的 w_equivalent，进行惩罚！
                    R_eff_sci = sigma_ind2_list{m} + (1 / w_equivalent) * sigma_dep2_list{m};

                    % 【防御性】：防除0
                    R_eff_sci = max(R_eff_sci, 1e-12);
                    
                    Xi_nm_sci = (1 / R_eff_sci) * U_list{m};
                    
                    % Lift 到 8D 状态空间
                    Lambda_sci = S_p' * Xi_nm_sci * S_p;
                    lambda_sci = S_p' * Xi_nm_sci * (x_ML - p_prior);
                    
                    % 纯加法更新 (此时Omega_anc的权重隐式视作了1，完美保护了速度和零偏)
                    Omega_fused = Omega_fused + Lambda_sci;
                    eta_fused = eta_fused + lambda_sci;
                end
            end
            
            % =============================================================
            % Step 5: State Extraction and Manifold Retraction
            % =============================================================
            % 强制对称，保障数值计算极度稳定
            Omega_fused = (Omega_fused + Omega_fused') / 2;
            
            % 【防御性】：检测矩阵的条件数，如果网络太差使得信息矩阵接近奇异，
            % 加上微弱的吉洪诺夫正则化（对角线微小偏移）强制其可逆。
            if cond(Omega_fused) > 1e10
                Omega_fused = Omega_fused + eye(8) * 1e-8;
            end
            
            % 提取后验协方差与误差均值
            P_post = Omega_fused \ eye(8);
            P_post = (P_post + P_post') / 2; 
            
            delta_x_post = P_post * eta_fused;
            
            % 【终极防御兜底 Fallback】：
            % 万一上面哪个地方浮点数爆炸了算出了 NaN，立刻退回到安全的 Anchor 结果
            if any(isnan(delta_x_post)) || any(isinf(delta_x_post))
                warning('DMLKF: NaN detected in delta_x_post! Fallback to Anchor-only update.');
                delta_x_post = delta_x_anc;
                P_post = P_anc;
            end
            
            % 最终回代折叠至非线性流形空间
            obj.P = P_post;
            obj.state = boxplus_Ms(state_prior, delta_x_post);
        end
    end
end
