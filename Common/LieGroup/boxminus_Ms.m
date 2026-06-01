function dx = boxminus_Ms(x1, x2)
    % 复合流形减法 (Eq. 14)
    
    d_xi = boxminus_SE2(x1.T, x2.T);
    d_ba = x1.ba - x2.ba;
    d_bw = x1.bw - x2.bw;
    
    dx = [d_xi; d_ba; d_bw];
end