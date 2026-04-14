%% ================= 主程序 =================
clc; clear;
%% 1. 数据加载与预处理
disp('=== 步骤1: 加载数据与预处理 ===');
IMUdata1 = readmatrix("IMUdata1.csv");
imu_raw = [IMUdata1(:,1), IMUdata1(:,26:31)];
t = imu_raw(:, 1);
acc_raw = imu_raw(:, 2:4) * 9.79362; 
gyro_raw = imu_raw(:, 5:7) * pi/180;
fs = 100; g_ref = 9.79362;

%% 2. 高精度静态区间检测
disp('=== 步骤2: 静态区间检测 ===');
static_regions = detectStaticRegions(acc_raw, gyro_raw, fs, 25);
num_regions = size(static_regions, 1);
fprintf('检测到 %d 个有效静态区间。\n', num_regions);

static_acc_means = zeros(num_regions, 3);
static_gyro_means = zeros(num_regions, 3);
for i = 1:num_regions
    idx = static_regions(i,1):static_regions(i,2);
    static_acc_means(i,:) = mean(acc_raw(idx,:));
    static_gyro_means(i,:) = mean(gyro_raw(idx,:));
end

%% 3. 加速度计标定 (12参)
disp('=== 步骤3: 加速度计标定 ===');
init_acc = [0,0,0, 1,1,1, 0,0,0, 0,0,0];
opts_acc = optimoptions('lsqnonlin', 'Algorithm','levenberg-marquardt', ...
    'Display','iter', 'FunctionTolerance',1e-9);
acc_params = lsqnonlin(@(p) acc_cost_function(p, static_acc_means, g_ref), ...
    init_acc, [], [], opts_acc);

acc_b = acc_params(1:3); acc_s = acc_params(4:6); acc_m = acc_params(7:12);
T_a = eye(3);
T_a(1,2)=-acc_m(1); T_a(1,3)=acc_m(2); T_a(2,1)=acc_m(3); T_a(2,3)=-acc_m(4);
T_a(3,1)=-acc_m(5); T_a(3,2)=acc_m(6);
K_a = diag(acc_s);

% 严格正演应用新校准模型: a_true = (a_meas - b) / (T*K)'
acc_calib = (acc_raw - acc_b) / (T_a * K_a)';
fprintf('加速度计标定完成。\n');

%% 4. 陀螺仪标定 (12参优化)
disp('=== 步骤4: 陀螺仪标定 ===');
gyro_bias_init = mean(static_gyro_means, 1); % 零偏预估计作为初始值

g_refs = zeros(num_regions, 3);
for i = 1:num_regions
    idx = static_regions(i,1):static_regions(i,2);
    g_refs(i, :) = mean(acc_calib(idx, :)); % 已校准的加速度即重力观测值
end

init_gyro = [gyro_bias_init, 1,1,1, 0,0,0, 0,0,0]; % 12参: [b, s, m]
opts_gyro = optimoptions('lsqnonlin', 'Algorithm','levenberg-marquardt', ...
    'Display','iter', 'FunctionTolerance',1e-9);
gyro_params = lsqnonlin(@(p) gyro_cost_function(p, gyro_raw, static_regions, ...
    g_refs, fs), init_gyro, [], [], opts_gyro);

gyro_b = gyro_params(1:3); gyro_s = gyro_params(4:6); gyro_m = gyro_params(7:12);
T_g = eye(3);
T_g(1,2)=-gyro_m(1); T_g(1,3)=gyro_m(2); T_g(2,1)=gyro_m(3); T_g(2,3)=-gyro_m(4);
T_g(3,1)=-gyro_m(5); T_g(3,2)=gyro_m(6);
K_g = diag(gyro_s);

% 严格正演应用新校准模型: omega_true = (omega_meas - b) / (T*K)'
gyro_calib = (gyro_raw - gyro_b) / (T_g * K_g)';
fprintf('陀螺仪标定完成。\n');


%% 5. 标定结果输出
disp('=== 标定参数输出 ===');
fprintf('【加速度计】\n');
fprintf('标度因数 (K^a): [%.6f, %.6f, %.6f]\n', acc_s);
fprintf('安装误差矩阵 (T^a):\n'); disp(T_a);
fprintf('零偏 (b^a): [%.6f, %.6f, %.6f]\n\n', acc_b);

fprintf('【陀螺仪】\n');
fprintf('标度因数 (K^g): [%.6f, %.6f, %.6f]\n', gyro_s);
fprintf('安装误差矩阵 (T^g):\n'); disp(T_g);
fprintf('零偏 (b^g) [优化结果]: [%.6f, %.6f, %.6f]\n', gyro_b);
cali_fig(t, acc_raw, acc_calib, gyro_raw, gyro_calib, static_regions, g_ref);

% 将校准后的数据输出到CSV文件
output  = [t, acc_calib, gyro_calib];
writematrix(output, 'output.csv', 'Delimiter', ',');

%% 6. 结果验证 (加速度计模长 + 陀螺仪重力矢量残差模长)
disp('=== 📈 结果验证 ===');

% 6.1 加速度计：模长RMSE
static_mask = false(size(t,1), 1);
for i = 1:num_regions
    static_mask(static_regions(i,1):static_regions(i,2)) = true;
end
% 标定前：直接使用原始数据
rmse_acc_raw = sqrt(mean((vecnorm(acc_raw(static_mask,:),2,2) - g_ref).^2));
% 标定后：使用新模型校准数据
rmse_acc_cal = sqrt(mean((vecnorm(acc_calib(static_mask,:),2,2) - g_ref).^2));
fprintf('加速度计模长 RMSE (标定前 -> 标定后): %.5f -> %.5f m/s^2\n', ...
    rmse_acc_raw, rmse_acc_cal);

% 6.2 陀螺仪：重力矢量残差模长 |a_g^k - a_hat_k|_2
% --- 标定前评估 (仅扣除零偏，尺度=1，失准=0) ---
gyro_bias_simple = mean(static_gyro_means, 1);
gyro_res_raw = [];
for i = 1:num_regions-1
    buf = round(0.2*fs);
    idx_s = max(1, static_regions(i,2)-buf);
    idx_e = min(size(gyro_raw,1), static_regions(i+1,1)+buf);
    if idx_e <= idx_s, continue; end
    
    gyro_seg = gyro_raw(idx_s:idx_e, :);
    gyro_simple = gyro_seg - gyro_bias_simple; % 新模型下 M=eye 时仅减零偏
    
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

% --- 标定后评估 (使用优化后的完整参数) ---
gyro_res_cal = [];
for i = 1:num_regions-1
    buf = round(0.2*fs);
    idx_s = max(1, static_regions(i,2)-buf);
    idx_e = min(size(gyro_calib,1), static_regions(i+1,1)+buf);
    if idx_e <= idx_s, continue; end
    
    gyro_seg = gyro_calib(idx_s:idx_e, :); % 已完整校准
    
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

fprintf('陀螺仪重力矢量残差模长 RMSE (标定前 -> 标定后): %.6f -> %.6f m/s^2\n', ...
    rmse_gyro_raw, rmse_gyro_cal);


disp('=== 校准流程结束 ===');