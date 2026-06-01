classdef PF_filter < EKF_filter
    properties
        num_particles
        P_state       % Np x 8: [x, y, theta, vx, vy, bax, bay, bw]
        weights       
    end
    
    methods
        function obj = PF_filter(Init_state, flag, use_ci, num_particles)
            if nargin < 4, num_particles = 450; end % 精度表现与DMLKF接近
            if nargin < 3, use_ci = false; end
            if nargin < 2, flag = 'general'; end
            if nargin < 1, Init_state = []; end
            
            obj@EKF_filter(Init_state, flag, use_ci);
            
            obj.num_particles = num_particles;
            obj.weights = ones(num_particles, 1) / num_particles; 
            
            % 用协方差采样粒子，生成一团和真实不确定性一致的初始粒子群
            obj.P_state = zeros(num_particles, 8);
            P_sym = (obj.P + obj.P') / 2; 
            [R_chol, p_chol] = chol(P_sym, 'lower');
            if p_chol ~= 0, R_chol = diag(sqrt(diag(P_sym))); end
            
            for i = 1:num_particles
                dx = R_chol * randn(8, 1);
                temp_state = boxplus_Ms(obj.state, dx); 
                
                th = atan2(temp_state.T(2,1), temp_state.T(1,1));
                obj.P_state(i, :) = [temp_state.T(1,4), temp_state.T(2,4), th, ...
                                     temp_state.T(1,3), temp_state.T(2,3), ...
                                     temp_state.ba(1), temp_state.ba(2), temp_state.bw];
            end
        end
        
        function predict(obj, am, wm, dt)
            Np = obj.num_particles;
            np = obj.noise_params;
            
            % 噪声膨胀系数
            k_explore = 2.5; 
            k_bias = 3.0; 
            
            % 过程噪声（加计、陀螺仪、偏置游走）
            na_x = randn(Np, 1) * np.sigma_na * k_explore;
            na_y = randn(Np, 1) * np.sigma_na * k_explore;
            nw   = randn(Np, 1) * np.sigma_nw * k_explore;
            
            nba_x = randn(Np, 1) * np.sigma_ba * sqrt(dt) * k_bias;
            nba_y = randn(Np, 1) * np.sigma_ba * sqrt(dt) * k_bias;
            nbw   = randn(Np, 1) * np.sigma_bw * sqrt(dt) * k_bias;
            
            % 读取状态
            Px = obj.P_state(:,1); Py = obj.P_state(:,2); Pth = obj.P_state(:,3);
            Pvx = obj.P_state(:,4); Pvy = obj.P_state(:,5);
            Pbax = obj.P_state(:,6); Pbay = obj.P_state(:,7); Pbw = obj.P_state(:,8);
            
            % 偏置量加上随机游走
            Pbax = Pbax + nba_x;
            Pbay = Pbay + nba_y;
            Pbw  = Pbw  + nbw;
            
            % 矫正加计陀螺仪输入
            ab_x = am(1) - Pbax - na_x;
            ab_y = am(2) - Pbay - na_y;
            wb   = wm    - Pbw  - nw;
            
            % 把机体坐标下的加速度转到世界坐标下
            cos_th = cos(Pth);
            sin_th = sin(Pth); 
            a_world_x = cos_th .* ab_x - sin_th .* ab_y;
            a_world_y = sin_th .* ab_x + cos_th .* ab_y;
            

            % 执行运动学离散更新
            Px  = Px  + Pvx * dt;
            Py  = Py  + Pvy * dt;
            
            Pvx = Pvx + a_world_x * dt;
            Pvy = Pvy + a_world_y * dt;
            
            Pth = Pth + wb * dt;
            Pth = mod(Pth + pi, 2*pi) - pi;
            
            obj.P_state = [Px, Py, Pth, Pvx, Pvy, Pbax, Pbay, Pbw];

            % 将粒子群计算成一个状态和协方差量
            obj.extract_mean_cov();
        end
        
        function update_anchor(obj, anc_meas, anc_pos)
            m_anc = length(anc_meas);
            if m_anc == 0, return; end
            
            Np = obj.num_particles;
            log_w_update = zeros(Np, 1);
            Px = obj.P_state(:,1); Py = obj.P_state(:,2);
            
            % 平滑系数，用于增大观测噪声方差，降低对数似然的尖锐
            smooth_factor = 3.0; 
            sigma2 = (obj.noise_params.sigma_anc * smooth_factor)^2;
            
            % 计算每一个基站测距提供的对数似然之和
            for j = 1:m_anc
                dist = sqrt((Px - anc_pos(j,1)).^2 + (Py - anc_pos(j,2)).^2);
                log_w_update = log_w_update - 0.5 * ((anc_meas(j) - dist).^2) / sigma2;
            end
            obj.apply_weights_and_resample(log_w_update);
        end
        
        function update_general(obj, rel_meas, neighbor_pos)
            if strcmp(obj.flag, 'anchor_only'), return; end
            valid_idx = find(~isnan(rel_meas));
            num_valid = length(valid_idx);
            if num_valid == 0, return; end
            
            if obj.use_ci
                w_rel = 1.0 / num_valid;
            else
                w_rel = 1.0;
            end
            
            Np = obj.num_particles;
            log_w_update = zeros(Np, 1);
            Px = obj.P_state(:,1); Py = obj.P_state(:,2);
            
            smooth_factor = 3.0; 
            sigma2 = (obj.noise_params.sigma_rel * smooth_factor)^2;
            
            for j = 1:num_valid
                idx = valid_idx(j);
                dist = sqrt((Px - neighbor_pos(idx, 1)).^2 + (Py - neighbor_pos(idx, 2)).^2);
                log_w_update = log_w_update - w_rel * 0.5 * ((rel_meas(idx) - dist).^2) / sigma2;
            end
            obj.apply_weights_and_resample(log_w_update);
        end
    end
    
    methods (Access = private)
        function apply_weights_and_resample(obj, log_w_update)
            log_w = log(obj.weights) + log_w_update;
            w = exp(log_w - max(log_w)); 
            if sum(w) == 0 || isnan(sum(w))
                obj.weights = ones(obj.num_particles, 1) / obj.num_particles;
            else
                obj.weights = w / sum(w);
            end
            
            % 计算有效粒子数
            N_eff = 1 / sum(obj.weights.^2);
            if N_eff < obj.num_particles * 0.8 % 有效粒子 < 65%时重采样
                obj.resample_particles();
            end
            obj.extract_mean_cov();
        end
        
        function resample_particles(obj)
            Np = obj.num_particles;
            edges = min([0; cumsum(obj.weights)], 1);
            edges(end) = 1; 
            
            u = (rand / Np) : (1/Np) : 1;
            [~, idx] = histc(u, edges);
            idx = min(max(idx, 1), Np);
            
            obj.P_state = obj.P_state(idx, :);
            obj.weights = ones(Np, 1) / Np;
            
            % 重采样后再加入噪声，避免粒子多样性过低
            obj.P_state(:, 4) = obj.P_state(:, 4) + randn(Np, 1) * 0.05;   
            obj.P_state(:, 5) = obj.P_state(:, 5) + randn(Np, 1) * 0.05;   
            obj.P_state(:, 6) = obj.P_state(:, 6) + randn(Np, 1) * 0.002;  
            obj.P_state(:, 7) = obj.P_state(:, 7) + randn(Np, 1) * 0.002;  
            obj.P_state(:, 8) = obj.P_state(:, 8) + randn(Np, 1) * 0.0002; 
        end
        
        function extract_mean_cov(obj)
            w = obj.weights;
            
            mean_x = sum(w .* obj.P_state(:,1));
            mean_y = sum(w .* obj.P_state(:,2));
            mean_vx = sum(w .* obj.P_state(:,4));
            mean_vy = sum(w .* obj.P_state(:,5));
            mean_bax = sum(w .* obj.P_state(:,6));
            mean_bay = sum(w .* obj.P_state(:,7));
            mean_bw = sum(w .* obj.P_state(:,8));
            
            mean_th = atan2(sum(w .* sin(obj.P_state(:,3))), sum(w .* cos(obj.P_state(:,3))));
            
            obj.state.T = eye(4);
            obj.state.T(1:2, 1:2) = [cos(mean_th), -sin(mean_th); sin(mean_th), cos(mean_th)];
            obj.state.T(1:2, 3) = [mean_vx; mean_vy];
            obj.state.T(1:2, 4) = [mean_x; mean_y];
            
            obj.state.ba = [mean_bax; mean_bay];
            obj.state.bw = mean_bw;
            
            % SE_2(2) 右扰动误差提取
            R_mean_inv = [cos(mean_th), sin(mean_th); -sin(mean_th), cos(mean_th)]; 
            
            dx = obj.P_state(:,1) - mean_x;
            dy = obj.P_state(:,2) - mean_y;
            d_rho_p = R_mean_inv * [dx, dy]'; % 位置切空间误差
            
            dvx = obj.P_state(:,4) - mean_vx;
            dvy = obj.P_state(:,5) - mean_vy;
            d_rho_v = R_mean_inv * [dvx, dvy]'; % 速度切空间误差
            
            d_th = obj.P_state(:,3) - mean_th;
            d_th = mod(d_th + pi, 2*pi) - pi;
            
            E = [d_th'; d_rho_v(1,:); d_rho_v(2,:); d_rho_p(1,:); d_rho_p(2,:); ...
                 obj.P_state(:,6)' - mean_bax; obj.P_state(:,7)' - mean_bay; obj.P_state(:,8)' - mean_bw];
            
            obj.P = E * (w .* E'); 
            obj.P = (obj.P + obj.P') / 2; 
        end
    end
end
