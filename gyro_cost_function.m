function err = gyro_cost_function(params, gyro_raw, static_regions, g_refs, fs)

    gyro_bias = params(1:3);
    s = params(4:6);
    m = params(7:12);
    

    T = eye(3);
    T(1,2) = -m(1); T(1,3) =  m(2);
    T(2,1) =  m(3); T(2,3) = -m(4);
    T(3,1) = -m(5); T(3,2) =  m(6);
    
    K = diag(s);
    M_TK = T * K; 
    
    err = [];
    N = size(static_regions, 1);
    
    for i = 1:N-1
        buf = round(0.2 * fs); 
        idx_s = max(1, static_regions(i, 2) - buf);
        idx_e = min(size(gyro_raw, 1), static_regions(i+1, 1) + buf);
        if idx_e <= idx_s, continue; end
        
        gyro_seg = gyro_raw(idx_s:idx_e, :);

        omega_O = (gyro_seg - gyro_bias) / M_TK';
        

        q = quaternion([1 0 0 0]); dt = 1/fs;
        for k = 1:size(omega_O, 1)-1
            w_mid = (omega_O(k,:) + omega_O(k+1,:)) / 2;
            q = q * quaternion(w_mid * dt, 'rotvec');
        end
        q = normalize(q);
        

        g_pred = rotateframe(q, g_refs(i, :));

        err = [err; g_pred - g_refs(i+1, :)];
    end
end