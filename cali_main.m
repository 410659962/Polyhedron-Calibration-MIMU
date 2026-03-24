clc;clear;

disp('=== 步骤1: 加载数据与预处理 ===');
load("IMUdata1.mat");
load("IMUdata2.mat");

imu_raw = [IMUdata2{:,1}, IMUdata2{:,26:31}];
t = imu_raw(:, 1);
acc_raw = imu_raw(:, 2:4) * 9.79362;%g->m/s^2
gyro_raw = imu_raw(:, 5:7) * pi/180;%deg/s->rad/s

fs = 100; 
disp(['数据加载完成: ' num2str(length(t)) ' 个采样点, 采样率 ' num2str(fs) 'Hz']);

disp('=== 步骤2: 静态区间检测 ===');

win_len = round(fs * 2); 
threshold_acc_var = 0.005; 
threshold_gyro_var = 0.001;

acc_mag = vecnorm(acc_raw, 2, 2);
gyro_mag = vecnorm(gyro_raw, 2, 2);

acc_var = movvar(acc_mag, win_len);
gyro_var = movvar(gyro_mag, win_len);

is_static = (acc_var < threshold_acc_var) & (gyro_var < threshold_gyro_var);

min_len_points = fs * 1;
static_regions = detectStaticRegions(is_static, min_len_points);

merge_gap_sec = 1.5; 
merge_gap_points = round(fs * merge_gap_sec);

if ~isempty(static_regions)
    merged_regions = static_regions(1, :);
    for i = 2:size(static_regions, 1)
        prev_end = merged_regions(end, 2);
        curr_start = static_regions(i, 1);
        gap = curr_start - prev_end;

        acc_prev = mean(acc_raw(merged_regions(end, 1):merged_regions(end, 2), :));
        acc_curr = mean(acc_raw(static_regions(i, 1):static_regions(i, 2), :));
        acc_diff = norm(acc_prev - acc_curr); 
  
        if gap <= merge_gap_points && acc_diff < 0.2 
            merged_regions(end, 2) = static_regions(i, 2);
        else
            merged_regions = [merged_regions; static_regions(i, :)];
        end
    end
    static_regions = merged_regions;
end

num_regions = size(static_regions, 1);
fprintf('检测到 %d 个静态区间。\n', num_regions);

static_acc_means = zeros(num_regions, 3);
static_gyro_means = zeros(num_regions, 3);

for i = 1:num_regions
    idx_range = static_regions(i, 1):static_regions(i, 2);
    static_acc_means(i, :) = mean(acc_raw(idx_range, :));
    static_gyro_means(i, :) = mean(gyro_raw(idx_range, :));
end

%加速度计校准 (非线性优化 - 椭球拟合)
disp('=== 步骤3: 加速度计校准 (估计零偏、尺度因子、失准角) ===');
% 误差模型: a_meas = T_a * diag(S_a) * (a_true + b_a)
% 目标: || a_true || = g (重力模长)

g_ref = 9.79362; % 当地重力加速度
init_params_acc = [0, 0, 0, ...           % 零偏 Bias (3)
                   1, 1, 1, ...           % 尺度因子 Scale (3)
                   0, 0, 0];              % 失准角 Misalignment (3, 简化为欧拉角小量)

options = optimoptions('lsqnonlin', 'Display', 'iter', 'TolFun', 1e-8);


acc_params = lsqnonlin(@(p) acc_cost_function(p, static_acc_means, g_ref), ...
            init_params_acc, [], [], options);


acc_bias = acc_params(1:3);
acc_scale = acc_params(4:6);
acc_misalign = acc_params(7:9);

% 构建校准矩阵
% 误差模型: a_meas = T_a * diag(S_a) * a_true + b_a
% 校正公式: a_true = inv(T_a * diag(S_a)) * (a_meas - b_a)
T_a = eye(3);
% 简单的小角度失准矩阵模型 (下三角形式)
T_a(2,1) = acc_misalign(1); T_a(3,1) = acc_misalign(2); T_a(3,2) = acc_misalign(3);
M_acc = inv(T_a * diag(acc_scale));

disp('加速度计参数:');
fprintf('零偏 (m/s^2): [%.6f, %.6f, %.6f]\n', acc_bias);
fprintf('尺度因子: [%.6f, %.6f, %.6f]\n', acc_scale);
fprintf('失准角 (rad): [%.6f, %.6f, %.6f]\n', acc_misalign);
disp(T_a);
% disp(M_acc);
% 应用加速度计校准
acc_calib = (acc_raw - acc_bias) * M_acc'; 


%陀螺仪校准 (利用加速度计参考)
disp('=== 步骤4: 陀螺仪校准 ===');

% 理想情况下，静止时陀螺仪输出应为0
% 使用所有静态区间的陀螺仪数据取平均作为零偏初始估计
gyro_bias_guess = mean(static_gyro_means, 1); 

% 去除零偏
% 尺度因子与失准角优化 (使用加速度计作为参考)

% 仅优化尺度因子和失准角
init_params_gyro = [gyro_bias_guess, 1, 1, 1, 0, 0, 0, 0, 0, 0]; 

lb = [gyro_bias_guess - 0.5, 0.95, 0.95, 0.95, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1];
ub = [gyro_bias_guess + 0.5, 1.05, 1.05, 1.05,  0.1,  0.1,  0.1,  0.1,  0.1,  0.1];

options_gyro = optimoptions('lsqnonlin', 'Display', 'iter', ...
                            'MaxFunctionEvaluations', 10000, ... 
                            'MaxIterations', 500, ...          
                            'FunctionTolerance', 1e-7, ...
                            'StepTolerance', 1e-7);

g_refs = zeros(num_regions, 3);
for i = 1:num_regions
    idx = static_regions(i,1):static_regions(i,2);
    g_mean = mean(acc_calib(idx, :));
    g_refs(i, :) = g_mean / norm(g_mean); 
end



gyro_params = lsqnonlin(@(p) gyro_cost_function(p, gyro_raw, g_refs, static_regions, fs), ...
                           init_params_gyro, [lb], [ub], options_gyro);


gyro_bias = gyro_params(1:3);
gyro_scale = gyro_params(4:6);
gyro_misalign = gyro_params(7:12);

% 构建陀螺仪校正矩阵
% 误差模型: ω_meas = T_g * diag(S_g) * ω_true + b_g
% 校正公式: ω_true = inv(T_g * diag(S_g)) * (ω_meas - b_g)
T_g = eye(3);
T_g(1,2) = gyro_misalign(1); T_g(1,3) = gyro_misalign(2);
T_g(2,1) = gyro_misalign(3); T_g(2,3) = gyro_misalign(4);
T_g(3,1) = gyro_misalign(5); T_g(3,2) = gyro_misalign(6);
M_gyro = inv(T_g * diag(gyro_scale));

disp('陀螺仪参数:');
fprintf('零偏 (rad/s): [%.6f, %.6f, %.6f]\n', gyro_bias);
fprintf('尺度因子: [%.6f, %.6f, %.6f]\n', gyro_scale);
fprintf('失准角 (rad): 包含6个交叉耦合参数，详见矩阵 T_g\n');
disp(T_g);

% 应用陀螺仪校准
gyro_calib = (gyro_raw - gyro_bias) * M_gyro';


disp('=== 步骤5: Allan方差噪声分析 ===');

[~, max_len_idx] = max(diff(static_regions, [], 2));
longest_region = static_regions(max_len_idx, :);
static_idx = longest_region(1):longest_region(2);

[avar_gyro, tau_gyro] = allanvar(gyro_calib(static_idx, :), 'octave', fs);
[avar_acc, tau_acc] = allanvar(acc_calib(static_idx, :), 'octave', fs);

% figure('Name', 'Allan方差分析');
% subplot(2,1,1); loglog(tau_gyro, sqrt(avar_gyro), 'LineWidth', 2); title('陀螺仪Allan方差'); grid on;
% subplot(2,1,2); loglog(tau_acc, sqrt(avar_acc), 'LineWidth', 2); title('加速度计Allan方差'); grid on;






disp('=== 步骤6: 标定算法验证 ===');
% filt_calib = imufilter('SampleRate', fs,...
%     'ReferenceFrame', 'ENU');
% [quat_calib, ~] = filt_calib(acc_calib, gyro_calib);
% 
% q0 = quat_calib(1);  % 初始姿态
% q_relative = conj(q0) .* quat_calib;  % 相对于初始姿态的变化
% 
% eul_calib = eulerd(q_relative, 'XYZ', 'frame');
% 
% figure('Name', '校准后姿态');
% plot(t, eul_calib);
% title('校准后姿态角 (Roll/Pitch/Yaw)'); legend('Roll', 'Pitch', 'Yaw'); grid on;


static_raw_norms = [];
for i = 1:size(static_regions, 1)
    idx = static_regions(i,1):static_regions(i,2);
    norms = vecnorm(acc_raw(idx, :), 2, 2);
    static_raw_norms = [static_raw_norms; norms];
end
acc_rmse_raw = sqrt(mean((static_raw_norms - g_ref).^2));
fprintf('加速度计标定前模长 RMSE: %.4f m/s^2\n', acc_rmse_raw);


static_calib_norms = [];
for i = 1:size(static_regions, 1)
    idx = static_regions(i,1):static_regions(i,2);
    norms = vecnorm(acc_calib(idx, :), 2, 2);
    static_calib_norms = [static_calib_norms; norms];
end
acc_rmse = sqrt(mean((static_calib_norms - g_ref).^2));
fprintf('加速度计标定后模长 RMSE: %.4f m/s^2\n', acc_rmse);

% 评估相邻静态区间的陀螺仪积分精度（校准前）
angle_errors_raw = zeros(num_regions-1, 1);
for i = 1:num_regions-1
    idx_start = static_regions(i, 2) - round(0.15*fs);
    idx_end = static_regions(i+1, 1) + round(0.15*fs);

    % 提取原始陀螺仪数据进行积分
    gyro_seg = gyro_raw(idx_start:idx_end, :);
    q = quaternion([1 0 0 0]);
    for k = 1:size(gyro_seg,1)-1
        omega = (gyro_seg(k,:) + gyro_seg(k+1,:)) / 2;
        q = q * quaternion(omega * (1/fs), 'rotvec');
    end

    g_pred = rotateframe(q, g_refs(i, :));
    g_true = g_refs(i+1, :);

    % 计算两个矢量之间的夹角 (度)
    cos_theta = dot(g_pred, g_true) / (norm(g_pred)*norm(g_true));
    cos_theta = max(min(cos_theta, 1), -1); % 防止浮点误差超出范围
    angle_errors_raw(i) = acosd(cos_theta);
end
fprintf('校准前陀螺仪单次翻转平均姿态误差: %.4f 度\n', mean(angle_errors_raw));


% 评估相邻静态区间的陀螺仪积分精度
angle_errors = zeros(num_regions-1, 1);
for i = 1:num_regions-1
    idx_start = static_regions(i, 2) - round(0.15*fs);
    idx_end = static_regions(i+1, 1) + round(0.15*fs);

    % 提取标定后的陀螺仪数据进行积分
    gyro_seg = gyro_calib(idx_start:idx_end, :);
    q = quaternion([1 0 0 0]);
    for k = 1:size(gyro_seg,1)-1
        omega = (gyro_seg(k,:) + gyro_seg(k+1,:)) / 2;
        q = q * quaternion(omega * (1/fs), 'rotvec');
    end

    g_pred = rotateframe(q, g_refs(i, :));
    g_true = g_refs(i+1, :);

    % 计算两个矢量之间的夹角 (度)
    cos_theta = dot(g_pred, g_true) / (norm(g_pred)*norm(g_true));
    cos_theta = max(min(cos_theta, 1), -1); % 防止浮点误差超出范围
    angle_errors(i) = acosd(cos_theta);
end
fprintf('校准后陀螺仪单次翻转平均姿态误差: %.4f 度\n', mean(angle_errors));


disp('=== 校准流程结束 ===');