function err_data = analyze_error(est_pos, true_pos, plot_flag)
    % analyze_error 计算位置估计误差
    % 输入:
    %   est_pos   - N x 2 的估计位置 [X, Y]
    %   true_pos  - N x 2 的真实位置 [X, Y]
    %   plot_flag - 0 不画图 (默认), 1 画图
    % 输出:
    %   err_data  - 包含 err_X, err_Y, err_Horizontal 的结构体
    
    if nargin < 3
        plot_flag = 0;
    end
    
    % 计算各项误差
    err_X = est_pos(:, 1) - true_pos(:, 1);
    err_Y = est_pos(:, 2) - true_pos(:, 2);
    err_Horizontal = sqrt(err_X.^2 + err_Y.^2);
    
    % 打包数据
    err_data.err_X = err_X;
    err_data.err_Y = err_Y;
    err_data.err_Horizontal = err_Horizontal;
    
    % 如果要求画图
    if plot_flag
        figure('Name', 'Position Error Analysis', 'Position',[200, 200, 800, 600]);
        
        subplot(3, 1, 1);
        plot(err_X, 'r-', 'LineWidth', 1.2);
        grid on; ylabel('X Error (m)');
        title('Position Error Analysis');
        
        subplot(3, 1, 2);
        plot(err_Y, 'g-', 'LineWidth', 1.2);
        grid on; ylabel('Y Error (m)');
        
        subplot(3, 1, 3);
        plot(err_Horizontal, 'b-', 'LineWidth', 1.2);
        grid on; ylabel('Horizontal Error (m)'); 
        xlabel('Time Step');
    end
end

