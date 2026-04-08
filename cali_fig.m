function cali_fig(t, acc_raw, acc_calib, gyro_raw, gyro_calib, static_regions, g_ref)
    num_regions = size(static_regions, 1);
    N = length(t);
    
    t_plot = (t - t(1)) / 1000;
    
    static_mask = false(N, 1);
    for i = 1:num_regions
        static_mask(static_regions(i,1):static_regions(i,2)) = true;
    end
    
    static_indicator = zeros(N, 1);
    static_indicator(static_mask) = 1;
    
    idx_end = max(1, N-99):N;
    
    legend_opts = {'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off'};
    
    inset_pos_fig2 = [0.67, 0.32, 0.2, 0.15];
    inset_pos_fig3 = [0.63, 0.32, 0.2, 0.15];
    inset_pos_fig4 = [0.32, 0.32, 0.245, 0.21];
    inset_pos_fig5 = [0.3, 0.70, 0.245, 0.21];
    inset_pos_fig6 = [0.63, 0.32, 0.24, 0.2];
    inset_pos_fig7 = [0.22, 0.68, 0.245, 0.21];
    inset_pos_fig8 = [0.62, 0.32, 0.245, 0.21];
    

    figure('Name', '静态检测与加速度数据');
    plot(t_plot, acc_raw(:,1), 'r', 'LineWidth', 1); hold on;
    plot(t_plot, acc_raw(:,2), 'g', 'LineWidth', 1);
    plot(t_plot, acc_raw(:,3), 'b', 'LineWidth', 1);
    plot(t_plot, static_indicator, 'm', 'LineWidth', 1.5);
    ylabel('加速度 (m/s^2)');
    xlabel('时间 (s)');
    legend('X轴', 'Y轴', 'Z轴', '静态检测', legend_opts{:});
    grid on;
    

    figure('Name', '加速度计X轴 - 标定前后对比');
    plot(t_plot, acc_raw(:,1), 'k', 'LineWidth', 1); hold on;
    plot(t_plot, acc_calib(:,1), 'r', 'LineWidth', 1.5);
    ylabel('X轴 (m/s^2)');
    xlabel('时间 (s)');
    legend('原始数据', '本方法标定后', legend_opts{:});
    grid on;
    axes('Position', inset_pos_fig2);
    plot(t_plot(idx_end), acc_raw(idx_end,1), 'k', 'LineWidth', 1); hold on;
    plot(t_plot(idx_end), acc_calib(idx_end,1), 'r', 'LineWidth', 1.5);
    grid on;
    box on;
    

    figure('Name', '加速度计Y轴 - 标定前后对比');
    plot(t_plot, acc_raw(:,2), 'k', 'LineWidth', 1); hold on;
    plot(t_plot, acc_calib(:,2), 'g', 'LineWidth', 1.5);
    ylabel('Y轴 (m/s^2)');
    xlabel('时间 (s)');
    legend('原始数据', '本方法标定后', legend_opts{:});
    grid on;
    axes('Position', inset_pos_fig3);
    plot(t_plot(idx_end), acc_raw(idx_end,2), 'k', 'LineWidth', 1); hold on;
    plot(t_plot(idx_end), acc_calib(idx_end,2), 'g', 'LineWidth', 1.5);
    grid on;
    box on;
    

    figure('Name', '加速度计Z轴 - 标定前后对比');
    plot(t_plot, acc_raw(:,3), 'k', 'LineWidth', 1); hold on;
    plot(t_plot, acc_calib(:,3), 'b', 'LineWidth', 1.5);
    ylabel('Z轴 (m/s^2)');
    xlabel('时间 (s)');
    legend('原始数据', '本方法标定后', legend_opts{:});
    grid on;
    axes('Position', inset_pos_fig4);
    plot(t_plot(idx_end), acc_raw(idx_end,3), 'k', 'LineWidth', 1); hold on;
    plot(t_plot(idx_end), acc_calib(idx_end,3), 'b', 'LineWidth', 1.5);
    grid on;
    box on;
    

    figure('Name', '标定前后加速度计模长对比');
    
    acc_norm_raw = vecnorm(acc_raw, 2, 2);
    acc_norm_calib = vecnorm(acc_calib, 2, 2);
    plot(t_plot, acc_norm_raw, 'r', 'LineWidth', 1); hold on;
    plot(t_plot, acc_norm_calib, 'b', 'LineWidth', 1.5);
    yline(g_ref, 'k', 'LineWidth', 1.5);
    ylabel('加速度模长 (m/s^2)');
    xlabel('时间 (s)');
    legend('原始数据', '本方法标定后', '重力参考值', legend_opts{:});
    grid on;
    axes('Position', inset_pos_fig5);
    plot(t_plot(idx_end), acc_norm_raw(idx_end), 'r', 'LineWidth', 1); hold on;
    plot(t_plot(idx_end), acc_norm_calib(idx_end), 'b', 'LineWidth', 1.5);
    yline(g_ref, 'k', 'LineWidth', 1.5);
    grid on;
    box on;
    

    figure('Name', '陀螺仪X轴 - 标定前后对比');
    plot(t_plot, gyro_raw(:,1)*180/pi, 'k', 'LineWidth', 1); hold on;
    plot(t_plot, gyro_calib(:,1)*180/pi, 'r', 'LineWidth', 1.5);
    ylabel('X轴 (deg/s)');
    xlabel('时间 (s)');
    legend('原始数据', '本方法标定后', legend_opts{:});
    grid on;
    axes('Position', inset_pos_fig6);
    plot(t_plot(idx_end), gyro_raw(idx_end,1)*180/pi, 'k', 'LineWidth', 1); hold on;
    plot(t_plot(idx_end), gyro_calib(idx_end,1)*180/pi, 'r', 'LineWidth', 1.5);
    grid on;
    box on;
    

    figure('Name', '陀螺仪Y轴 - 标定前后对比');
    plot(t_plot, gyro_raw(:,2)*180/pi, 'k', 'LineWidth', 1); hold on;
    plot(t_plot, gyro_calib(:,2)*180/pi, 'g', 'LineWidth', 1.5);
    ylabel('Y轴 (deg/s)');
    xlabel('时间 (s)');
    legend('原始数据', '本方法标定后', legend_opts{:});
    grid on;
    axes('Position', inset_pos_fig7);
    plot(t_plot(idx_end), gyro_raw(idx_end,2)*180/pi, 'k', 'LineWidth', 1); hold on;
    plot(t_plot(idx_end), gyro_calib(idx_end,2)*180/pi, 'g', 'LineWidth', 1.5);
    grid on;
    box on;
    

    figure('Name', '陀螺仪Z轴 - 标定前后对比');
    plot(t_plot, gyro_raw(:,3)*180/pi, 'k', 'LineWidth', 1); hold on;
    plot(t_plot, gyro_calib(:,3)*180/pi, 'b', 'LineWidth', 1.5);
    ylabel('Z轴 (deg/s)');
    xlabel('时间 (s)');
    legend('原始数据', '本方法标定后', legend_opts{:});
    grid on;
    axes('Position', inset_pos_fig8);
    plot(t_plot(idx_end), gyro_raw(idx_end,3)*180/pi, 'k', 'LineWidth', 1); hold on;
    plot(t_plot(idx_end), gyro_calib(idx_end,3)*180/pi, 'b', 'LineWidth', 1.5);
    grid on;
    box on;
end
