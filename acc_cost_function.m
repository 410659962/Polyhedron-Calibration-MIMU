function err = acc_cost_function(params, acc_means, g_ref)
    % 加速度计代价函数
    % 误差模型: a_meas = T_a * diag(S_a) * a_true + b_a
    % 校正公式: a_true = inv(T_a * diag(S_a)) * (a_meas - b_a)
    % params = [bias_x, bias_y, bias_z, scale_x, scale_y, scale_z, mis_x, mis_y, mis_z]
    bias = params(1:3);
    scale = params(4:6);
    mis = params(7:9);
    
    T = eye(3);
    T(2,1) = mis(1); T(3,1) = mis(2); T(3,2) = mis(3);
    
    Ka = diag(scale);
    M_acc = inv(T * Ka);
    
    acc_calib = (acc_means - bias) * M_acc';
    
    acc_norms = sqrt(sum(acc_calib.^2, 2));
    err = acc_norms - g_ref;
end