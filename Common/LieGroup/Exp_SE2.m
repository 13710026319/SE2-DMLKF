function T = Exp_SE2(xi)
    % Exp_SE2: 映射 se_2(2) 李代数到 SE_2(2) 李群矩阵
    % xi: 5x1 向量 [phi; rho_v(2x1); rho_p(2x1)]
    % T:  4x4 SE_2(2) 变换矩阵 (Eq. 6)
    
    phi = xi(1);
    rho_v = xi(2:3);
    rho_p = xi(4:5);
    
    R = [cos(phi), -sin(phi); 
         sin(phi),  cos(phi)];
    
    % 计算左雅可比 V(phi) (Eq. 7)
    if abs(phi) > 1e-7
        V = (1 / phi) * [sin(phi), -(1 - cos(phi)); 
                         1 - cos(phi), sin(phi)];
    else
        V = eye(2);
    end
    
    % 拼装 4x4 SE_2(2) 矩阵
    T = eye(4);
    T(1:2, 1:2) = R;
    T(1:2, 3)   = V * rho_v;  % 对应速度位移
    T(1:2, 4)   = V * rho_p;  % 对应位置位移
end