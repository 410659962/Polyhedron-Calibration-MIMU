%% ================= 多MIMU校准主程序 =================
clc; clear;
%% 1. 数据加载与预处理
disp('=== 步骤1: 加载数据与预处理 ===');
IMUdata1 = readmatrix("IMUdata1.csv");
t = IMUdata1(:, 1);
fs = 100; g_ref = 9.79362;
num_imus = 9;

for i = 1:num_imus
    acc_col_start = (i-1)*6 + 2;
    gyro_col_start = (i-1)*6 + 5;
    acc_raw{i} = IMUdata1(:, acc_col_start:acc_col_start+2) * g_ref;
    gyro_raw{i} = IMUdata1(:, gyro_col_start:gyro_col_start+2) * pi/180;
end
fprintf('已加载 %d 个MIMU的数据。\n', num_imus);

%% 2. 静态区间检测 (使用MIMU1的数据检测，所有MIMU共用)
disp('=== 步骤2: 静态区间检测 ===');
static_regions = detectStaticRegions(acc_raw{1}, gyro_raw{1}, fs, 25);
num_regions = size(static_regions, 1);
fprintf('检测到 %d 个有效静态区间。\n', num_regions);

%% 3. 预分配存储结构
calib_params = struct();
for i = 1:num_imus
    calib_params(i).acc_b = zeros(1,3);
    calib_params(i).acc_s = ones(1,3);
    calib_params(i).acc_T = eye(3);
    calib_params(i).gyro_b = zeros(1,3);
    calib_params(i).gyro_s = ones(1,3);
    calib_params(i).gyro_T = eye(3);
    calib_params(i).acc_calib = zeros(size(acc_raw{1}));
    calib_params(i).gyro_calib = zeros(size(gyro_raw{1}));
end

%% 4. 循环校准每个MIMU
for imu_idx = 1:num_imus
    fprintf('\n========== 校准 MIMU %d ==========\n', imu_idx);
    
    %% 4.1 计算静态区间均值
    static_acc_means = zeros(num_regions, 3);
    static_gyro_means = zeros(num_regions, 3);
    for i = 1:num_regions
        idx = static_regions(i,1):static_regions(i,2);
        static_acc_means(i,:) = mean(acc_raw{imu_idx}(idx,:));
        static_gyro_means(i,:) = mean(gyro_raw{imu_idx}(idx,:));
    end
    
    %% 4.2 加速度计标定 (12参)
    fprintf('--- 加速度计标定 ---\n');
    init_acc = [0,0,0, 1,1,1, 0,0,0, 0,0,0];
    opts_acc = optimoptions('lsqnonlin', 'Algorithm','levenberg-marquardt', ...
        'Display','off', 'FunctionTolerance',1e-9);
    acc_params = lsqnonlin(@(p) acc_cost_function(p, static_acc_means, g_ref), ...
        init_acc, [], [], opts_acc);
    
    acc_b = acc_params(1:3); acc_s = acc_params(4:6); acc_m = acc_params(7:12);
    T_a = eye(3);
    T_a(1,2)=-acc_m(1); T_a(1,3)=acc_m(2); T_a(2,1)=acc_m(3); T_a(2,3)=-acc_m(4);
    T_a(3,1)=-acc_m(5); T_a(3,2)=acc_m(6);
    K_a = diag(acc_s);
    
    acc_calib = (acc_raw{imu_idx} - acc_b) / (T_a * K_a)';
    
    calib_params(imu_idx).acc_b = acc_b;
    calib_params(imu_idx).acc_s = acc_s;
    calib_params(imu_idx).acc_T = T_a;
    calib_params(imu_idx).acc_calib = acc_calib;
    fprintf('MIMU%d 加速度计标定完成。\n', imu_idx);
    
    %% 4.3 陀螺仪标定 (12参优化)
    fprintf('--- 陀螺仪标定 ---\n');
    gyro_bias_init = mean(static_gyro_means, 1);
    
    g_refs = zeros(num_regions, 3);
    for i = 1:num_regions
        idx = static_regions(i,1):static_regions(i,2);
        g_refs(i, :) = mean(acc_calib(idx, :));
    end
    
    init_gyro = [gyro_bias_init, 1,1,1, 0,0,0, 0,0,0];
    opts_gyro = optimoptions('lsqnonlin', 'Algorithm','levenberg-marquardt', ...
        'Display','off', 'FunctionTolerance',1e-9);
    gyro_params = lsqnonlin(@(p) gyro_cost_function(p, gyro_raw{imu_idx}, static_regions, ...
        g_refs, fs), init_gyro, [], [], opts_gyro);
    
    gyro_b = gyro_params(1:3); gyro_s = gyro_params(4:6); gyro_m = gyro_params(7:12);
    T_g = eye(3);
    T_g(1,2)=-gyro_m(1); T_g(1,3)=gyro_m(2); T_g(2,1)=gyro_m(3); T_g(2,3)=-gyro_m(4);
    T_g(3,1)=-gyro_m(5); T_g(3,2)=gyro_m(6);
    K_g = diag(gyro_s);
    
    gyro_calib = (gyro_raw{imu_idx} - gyro_b) / (T_g * K_g)';
    
    calib_params(imu_idx).gyro_b = gyro_b;
    calib_params(imu_idx).gyro_s = gyro_s;
    calib_params(imu_idx).gyro_T = T_g;
    calib_params(imu_idx).gyro_calib = gyro_calib;
    fprintf('MIMU%d 陀螺仪标定完成。\n', imu_idx);
end

%% 5. 标定结果输出
disp(' ');
disp('=== 所有MIMU标定参数输出 ===');
for imu_idx = 1:num_imus
    fprintf('\n【MIMU %d】\n', imu_idx);
    fprintf('  加速度计零偏 (b^a): [%.6f, %.6f, %.6f]\n', calib_params(imu_idx).acc_b);
    fprintf('  加速度计标度因数 (K^a): [%.6f, %.6f, %.6f]\n', calib_params(imu_idx).acc_s);
    fprintf('  陀螺仪零偏 (b^g): [%.6f, %.6f, %.6f]\n', calib_params(imu_idx).gyro_b);
    fprintf('  陀螺仪标度因数 (K^g): [%.6f, %.6f, %.6f]\n', calib_params(imu_idx).gyro_s);
end

%% 6. 结果验证
disp(' ');
disp('===结果验证 ===');
static_mask = false(size(t,1), 1);
for i = 1:num_regions
    static_mask(static_regions(i,1):static_regions(i,2)) = true;
end

fprintf('\n%-8s | %-20s | %-20s\n', 'MIMU', 'Acc RMSE (前->后)', 'Gyro RMSE (前->后)');
fprintf('%s\n', repmat('-',1,60));

for imu_idx = 1:num_imus
    rmse_acc_raw = sqrt(mean((vecnorm(acc_raw{imu_idx}(static_mask,:),2,2) - g_ref).^2));
    rmse_acc_cal = sqrt(mean((vecnorm(calib_params(imu_idx).acc_calib(static_mask,:),2,2) - g_ref).^2));
    
    gyro_bias_simple = mean(gyro_raw{imu_idx}(static_mask,:), 1);
    g_refs = zeros(num_regions, 3);
    for i = 1:num_regions
        idx = static_regions(i,1):static_regions(i,2);
        g_refs(i, :) = mean(calib_params(imu_idx).acc_calib(idx, :));
    end
    
    gyro_res_raw = [];
    for i = 1:num_regions-1
        buf = round(0.2*fs);
        idx_s = max(1, static_regions(i,2)-buf);
        idx_e = min(size(gyro_raw{imu_idx},1), static_regions(i+1,1)+buf);
        if idx_e <= idx_s, continue; end
        
        gyro_seg = gyro_raw{imu_idx}(idx_s:idx_e, :);
        gyro_simple = gyro_seg - gyro_bias_simple;
        
        q = quaternion([1 0 0 0]);
        for k = 1:size(gyro_simple,1)-1
            w = (gyro_simple(k,:)+gyro_simple(k+1,:))/2;
            q = q * quaternion(w*(1/fs), 'rotvec');
        end
        q = normalize(q);
        g_pred = rotateframe(q, g_refs(i,:));
        res_vec = g_pred - g_refs(i+1,:);
        gyro_res_raw = [gyro_res_raw; norm(res_vec)];
    end
    rmse_gyro_raw = sqrt(mean(gyro_res_raw.^2));
    
    gyro_res_cal = [];
    for i = 1:num_regions-1
        buf = round(0.2*fs);
        idx_s = max(1, static_regions(i,2)-buf);
        idx_e = min(size(calib_params(imu_idx).gyro_calib,1), static_regions(i+1,1)+buf);
        if idx_e <= idx_s, continue; end
        
        gyro_seg = calib_params(imu_idx).gyro_calib(idx_s:idx_e, :);
        
        q = quaternion([1 0 0 0]);
        for k = 1:size(gyro_seg,1)-1
            w = (gyro_seg(k,:)+gyro_seg(k+1,:))/2;
            q = q * quaternion(w*(1/fs), 'rotvec');
        end
        q = normalize(q);
        g_pred = rotateframe(q, g_refs(i,:));
        res_vec = g_pred - g_refs(i+1,:);
        gyro_res_cal = [gyro_res_cal; norm(res_vec)];
    end
    rmse_gyro_cal = sqrt(mean(gyro_res_cal.^2));
    
    fprintf('MIMU %-3d | %.5f -> %.5f | %.6f -> %.6f\n', ...
        imu_idx, rmse_acc_raw, rmse_acc_cal, rmse_gyro_raw, rmse_gyro_cal);
end

%% 7. 输出校准后数据到CSV
output = zeros(size(t,1), 1 + 6*num_imus);
output(:, 1) = t;
for imu_idx = 1:num_imus
    col_start = 2 + (imu_idx-1)*6;
    output(:, col_start:col_start+2) = calib_params(imu_idx).acc_calib;
    output(:, col_start+3:col_start+5) = calib_params(imu_idx).gyro_calib;
end
writematrix(output, 'multi_output.csv', 'Delimiter', ',');
fprintf('\n校准后数据已保存至 multi_output.csv\n');

%% 8. 保存标定参数
save('calib_params.mat', 'calib_params', 'static_regions', 'fs', 'g_ref');
fprintf('标定参数已保存至 calib_params.mat\n');

disp('=== 多MIMU校准流程结束 ===');
