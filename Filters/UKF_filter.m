classdef UKF_filter < EKF_filter
    % UKF_filter: 基于流形误差空间的无迹卡尔曼滤波器 (Error-State UKF) 误差状态+SE2 
    % CI方案是仅对相对测距的噪声协方差除1/N，进行放大，这其实是类似于自适应，并不算是CI

    properties
        ukf_params  
        W_m
        W_c
        lambda
    end
    
    methods
        function obj = UKF_filter(Init_state, flag, use_ci, ukf_params)
            if nargin < 4 || isempty(ukf_params)
                ukf_params.alpha = 1e-3; 
                ukf_params.beta  = 2;    
                ukf_params.kappa = 0;    
            end
            if nargin < 3, use_ci = false; end
            if nargin < 2, flag = 'general'; end
            if nargin < 1, Init_state = []; end
            
            obj@EKF_filter(Init_state, flag, use_ci);
            
            obj.ukf_params = ukf_params;
            
            L = 8; 
            alpha = obj.ukf_params.alpha;
            obj.lambda = alpha^2 * (L + obj.ukf_params.kappa) - L;
            
            obj.W_m = zeros(2*L + 1, 1);
            obj.W_c = zeros(2*L + 1, 1);
            
            obj.W_m(1) = obj.lambda / (L + obj.lambda);
            obj.W_c(1) = obj.W_m(1) + (1 - alpha^2 + obj.ukf_params.beta);
            
            for i = 2:(2*L + 1)
                obj.W_m(i) = 1 / (2 * (L + obj.lambda));
                obj.W_c(i) = 1 / (2 * (L + obj.lambda));
            end
        end
        
        function update_anchor(obj, anc_meas, anc_pos)
            m_anc = length(anc_meas);
            if m_anc == 0, return; end
            
            R_anc = eye(m_anc) * (obj.noise_params.sigma_anc^2);
            obj.ut_measurement_update(anc_meas, anc_pos, R_anc);
        end
        
        function update_general(obj, rel_meas, neighbor_pos)
            if strcmp(obj.flag, 'anchor_only'), return; end
            
            valid_idx = find(~isnan(rel_meas));
            num_valid = length(valid_idx);
            if num_valid == 0, return; end
            
            meas_valid = rel_meas(valid_idx);
            pos_valid  = neighbor_pos(valid_idx, :);
            
            R_rel = eye(num_valid) * (obj.noise_params.sigma_rel^2);
            
            if obj.use_ci
                w_m = 1.0 / num_valid;
                R_rel = R_rel / w_m; 
            end
            
            obj.ut_measurement_update(meas_valid, pos_valid, R_rel);
        end
    end
    
    methods (Access = private)
        function ut_measurement_update(obj, z_meas, ref_pos, R)
            L = 8;
            m = length(z_meas);
            
            P_sym = (obj.P + obj.P') / 2; 
            [S, p_chol] = chol((L + obj.lambda) * P_sym, 'lower');
            if p_chol > 0
                [S, ~] = chol((L + obj.lambda) * P_sym + eye(L)*1e-9, 'lower');
            end
            
            delta_X = [zeros(L, 1), S, -S]; 
            Z_pred = zeros(m, 2*L + 1); 
            
            for i = 1:(2*L + 1)
                state_i = boxplus_Ms(obj.state, delta_X(:, i));
                pos_i = state_i.T(1:2, 4); % 位置在第4列
                
                for j = 1:m
                    Z_pred(j, i) = norm(pos_i - ref_pos(j, :)');
                end
            end
            
            z_hat = Z_pred * obj.W_m; 
            Z_diff = Z_pred - z_hat;  
            
            P_zz = Z_diff * diag(obj.W_c) * Z_diff' + R;
            P_xz = delta_X * diag(obj.W_c) * Z_diff';
            
            K = P_xz / P_zz;
            dx_post = K * (z_meas - z_hat);
            
            obj.P = obj.P - K * P_zz * K';
            obj.P = (obj.P + obj.P') / 2; 
            
            obj.state = boxplus_Ms(obj.state, dx_post);
        end
    end
end
