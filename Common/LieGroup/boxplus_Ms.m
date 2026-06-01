function x_new = boxplus_Ms(x, dx)
    % 复合流形加法 (Eq. 13)
    % x:  包含 T(4x4), ba(2x1), bw(1x1) 的结构体 (注：速度已被融合进 T)
    % dx: 8x1 误差状态向量 [d_xi(5x1); d_ba(2x1); d_bw(1x1)]
    
    d_xi = dx(1:5);
    d_ba = dx(6:7);
    d_bw = dx(8);
    
    x_new.T  = boxplus_SE2(x.T, d_xi);
    x_new.ba = x.ba + d_ba;
    x_new.bw = x.bw + d_bw;
end