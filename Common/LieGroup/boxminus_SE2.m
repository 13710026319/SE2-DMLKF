function xi = boxminus_SE2(T1, T2)
    % xi = Log(T2^-1 * T1) (Eq. 12)
    xi = Log_SE2(Inv_SE2(T2) * T1);
end