function T_new = boxplus_SE2(T, xi)
    % T_new = T * Exp(xi) (Eq. 11)
    T_new = T * Exp_SE2(xi);
end