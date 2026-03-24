function err = gyro_cost_function(params, gyro_raw, g_refs, static_regions, fs)
    % 陀螺仪代价函数
    % 误差模型: ω_meas = T_g * diag(S_g) * ω_true + b_g
    % 校正公式: ω_true = inv(T_g * diag(S_g)) * (ω_meas - b_g)
    % 输入 params 包含 12 个参数: [bias(3), scale(3), misalign(6)]
    
    bias = params(1:3);
    scale = params(4:6);
    mis = params(7:12); 
    
    T_g = eye(3);
    T_g(1,2) = mis(1); T_g(1,3) = mis(2);
    T_g(2,1) = mis(3); T_g(2,3) = mis(4);
    T_g(3,1) = mis(5); T_g(3,2) = mis(6);
    
    M_g = inv(T_g * diag(scale));
    
    err = [];
    N = size(static_regions, 1);
    
    for i = 1:N-1
        buffer = round(0.15 * fs); 
        idx_start = max(1, static_regions(i, 2) - buffer);
        idx_end = min(size(gyro_raw, 1), static_regions(i+1, 1) + buffer);
        
        if idx_end <= idx_start
            continue; 
        end
        
        gyro_segment = gyro_raw(idx_start:idx_end, :);
        gyro_corr = (gyro_segment - bias) * M_g';
        
        q = quaternion([1 0 0 0]); 
        dt = 1/fs;
        
        for k = 1:size(gyro_corr, 1)-1
            omega_mid = (gyro_corr(k, :) + gyro_corr(k+1, :)) / 2;
            dq = quaternion(omega_mid * dt, 'rotvec');
            q = q * dq;
        end
        
        g_pred = rotateframe(q, g_refs(i, :));
        res = g_pred - g_refs(i+1, :);
        err = [err; res(:)];
    end
end