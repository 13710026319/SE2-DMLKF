classdef EKF_filter < handle
% 误差流形EKF

    properties
        state       % struct 包含: T (4x4, SE_2(2)), ba (2x1), bw (1x1)
        P           % 误差协方差 (8x8)
        noise_params 
        flag         
        use_ci       
    end
    
    methods
        function obj = EKF_filter(Init_state, flag, use_ci)
            if nargin < 3, obj.use_ci = false; else, obj.use_ci = use_ci; end
            if nargin < 2, obj.flag = 'anchor_only'; else, obj.flag = flag; end
            
            % 速度融合进 T 的第3列，位置在第4列
            default_state.T  = eye(4);
            default_state.ba = [0; 0];
            default_state.bw = 0;
            
            default_noise.sigma_na = 0.05;
            default_noise.sigma_nw = 0.005;
            default_noise.sigma_ba = 0.002;
            default_noise.sigma_bw = 0.0002;
            default_noise.sigma_anc = 0.06;
            default_noise.sigma_rel = 0.06; 
            
            if nargin >= 1 && ~isempty(Init_state)
                if isfield(Init_state, 'state'), obj.state = Init_state.state; else, obj.state = default_state; end
                if isfield(Init_state, 'P'), obj.P = Init_state.P; else, obj.P = eye(8) * 0.1; end
                if isfield(Init_state, 'noise_params'), obj.noise_params = Init_state.noise_params; else, obj.noise_params = default_noise; end
            else
                obj.state = default_state;
                obj.P = eye(8) * 0.1;
                obj.noise_params = default_noise;
            end
        end
        
        function predict(obj, am, wm, dt)
            % SE_2(2) IMU 预测步 (Eq. 20 - 25, Eq. 32 - 34)
            R_hat = obj.state.T(1:2, 1:2);
            v_hat = obj.state.T(1:2, 3);
            ba_hat = obj.state.ba;
            bw_hat = obj.state.bw;
            
            w_b = wm - bw_hat;
            a_b = am - ba_hat;
            
            % 1. 标称更新
            nu = [w_b; a_b; R_hat' * v_hat];
            obj.state.T = obj.state.T * Exp_SE2(dt * nu);
            
            % 2. 状态无关连续时间雅可比 A, G 构建
            J = [0, -1; 1, 0];
            
            % 构建 A 矩阵 (Eq. 32)
            A = zeros(8,8);
            A(1, 8)     = -1;
            A(2:3, 1)   = J * a_b;
            A(2:3, 2:3) = -w_b * J;
            A(2:3, 6:7) = -eye(2);
            A(4:5, 2:3) = eye(2);
            A(4:5, 4:5) = -w_b * J;
            
            % 构建 G 矩阵 (Eq. 33)
            G = zeros(8,6);
            G(1, 3)     = -1;
            G(2:3, 1:2) = -eye(2);
            G(6:7, 4:5) = eye(2);
            G(8, 6)     = 1;
            
            % 3. 离散化 (Eq. 34)
            Fx = eye(8) + A * dt;
            Fw = G * dt;
            
            Q = diag([obj.noise_params.sigma_na^2, obj.noise_params.sigma_na^2, obj.noise_params.sigma_nw^2, ...
                      obj.noise_params.sigma_ba^2, obj.noise_params.sigma_ba^2, obj.noise_params.sigma_bw^2]);
                  
            % 4. 协方差传播
            obj.P = Fx * obj.P * Fx' + Fw * Q * Fw';
        end
        
        function update_anchor(obj, anc_meas, anc_pos)
            m = length(anc_meas);
            if m == 0, return; end 
            
            p_hat = obj.state.T(1:2, 4); % 位置在第4列
            R_hat = obj.state.T(1:2, 1:2);
            
            % 全新的降维算子 Sp (Eq. 37)
            S_p = [zeros(2, 3), R_hat, zeros(2, 3)]; 
            
            H_anc = zeros(m, 8);
            r_anc = zeros(m, 1);
            
            for i = 1:m
                a_i = anc_pos(i, :)';
                delta_p = p_hat - a_i;
                dist_hat = norm(delta_p);
                r_anc(i) = anc_meas(i) - dist_hat;
                
                c_i_T = delta_p' / dist_hat; 
                H_anc(i, :) = c_i_T * S_p;
            end
            
            R_anc = eye(m) * (obj.noise_params.sigma_anc^2);
            S = H_anc * obj.P * H_anc' + R_anc;
            K = obj.P * H_anc' / S;
            dx = K * r_anc;
            
            I_KH = eye(8) - K * H_anc;
            obj.P = I_KH * obj.P * I_KH' + K * R_anc * K';
            obj.P = (obj.P + obj.P') / 2;
            
            obj.state = boxplus_Ms(obj.state, dx);
        end
        
        function update_general(obj, rel_meas, neighbor_pos)
            if strcmp(obj.flag, 'anchor_only'), return; end
            
            m = length(rel_meas);
            if m == 0, return; end
            
            p_hat = obj.state.T(1:2, 4);
            R_hat = obj.state.T(1:2, 1:2);
            S_p = [zeros(2, 3), R_hat, zeros(2, 3)]; 
            
            H_rel = zeros(m, 8);
            r_rel = zeros(m, 1);
            
            for i = 1:m
                if isnan(rel_meas(i)), continue; end 
                n_i = neighbor_pos(i, :)';
                delta_p = p_hat - n_i;
                dist_hat = norm(delta_p);
                r_rel(i) = rel_meas(i) - dist_hat;
                
                c_i_T = delta_p' / dist_hat;
                H_rel(i, :) = c_i_T * S_p;
            end
            
            valid_idx = any(H_rel, 2); 
            H_rel = H_rel(valid_idx, :);
            r_rel = r_rel(valid_idx);
            m_valid = sum(valid_idx);
            
            if m_valid == 0, return; end
            
            R_rel = eye(m_valid) * (obj.noise_params.sigma_rel^2);
            if obj.use_ci
                w_m = 1.0 / m_valid;
                R_rel = R_rel / w_m; 
            end
            
            S = H_rel * obj.P * H_rel' + R_rel;
            K = obj.P * H_rel' / S;
            dx = K * r_rel;
            
            I_KH = eye(8) - K * H_rel;
            obj.P = I_KH * obj.P * I_KH' + K * R_rel * K';
            obj.P = (obj.P + obj.P') / 2;
            
            obj.state = boxplus_Ms(obj.state, dx);
        end
    end
end
