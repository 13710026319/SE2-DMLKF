function T_inv = Inv_SE2(T)
    % Inv_SE2: 求 SE_2(2) 矩阵的逆 (Eq. 10)
    % T:     4x4 SE_2(2) 矩阵
    % T_inv: 4x4 逆矩阵
    
    R = T(1:2, 1:2);
    v = T(1:2, 3);
    p = T(1:2, 4);
    
    T_inv = eye(4);
    T_inv(1:2, 1:2) = R';
    T_inv(1:2, 3)   = -R' * v;
    T_inv(1:2, 4)   = -R' * p;
end