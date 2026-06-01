function xi = Log_SE2(T)
    % Log_SE2: 映射 SE_2(2) 李群矩阵到 se_2(2) 李代数
    % T:  4x4 SE_2(2) 变换矩阵
    % xi: 5x1 向量 [phi; rho_v; rho_p] (Eq. 9)
    
    R = T(1:2, 1:2);
    v = T(1:2, 3);
    p = T(1:2, 4);
    
    phi = atan2(R(2,1), R(1,1));
    
    if abs(phi) > 1e-7
        V = (1 / phi) * [sin(phi), -(1 - cos(phi)); 
                         1 - cos(phi), sin(phi)];
        V_inv = inv(V); 
    else
        V_inv = eye(2);
    end
    
    rho_v = V_inv * v;
    rho_p = V_inv * p;
    
    xi = [phi; rho_v; rho_p];
end