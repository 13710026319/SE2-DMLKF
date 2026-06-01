function Ad_T = Adjoint_SE2(T)
    % Adjoint_SE2: 计算 SE_2(2) 的 5x5 伴随矩阵 (Eq. 15)
    % T:    4x4 SE_2(2) 矩阵
    % Ad_T: 5x5 伴随矩阵
    
    R = T(1:2, 1:2);
    v = T(1:2, 3);
    p = T(1:2, 4);
    
    % 叉乘算子 (Eq. 16)
    v_odot = [v(2); -v(1)];
    p_odot = [p(2); -p(1)];
          
    Ad_T = zeros(5, 5);
    Ad_T(1, 1)       = 1;
    Ad_T(2:3, 1)     = v_odot;
    Ad_T(2:3, 2:3)   = R;
    Ad_T(4:5, 1)     = p_odot;
    Ad_T(4:5, 4:5)   = R;
end