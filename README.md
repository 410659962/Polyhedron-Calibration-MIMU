# IMU Calibration

基于 MATLAB 的惯性测量单元 (MIMU) 校准工具，支持加速度计和陀螺仪的系统性误差标定。

## 功能特性

- **静态区间检测**：基于滑动窗口方差自动检测静止状态
- **加速度计校准**：估计零偏、尺度因子、失准角（椭球拟合）
- **陀螺仪校准**：利用重力矢量参考进行多参数联合优化
- **Allan 方差分析**：评估传感器随机噪声特性

## 项目结构

```
IMU_calibration/
├── cali_main.m              # 主校准脚本
├── data_pre.m               # 数据预处理
├── acc_cost_function.m      # 加速度计代价函数
├── gyro_cost_function.m     # 陀螺仪代价函数
├── detectStaticRegions.m    # 静态区间检测
├── cali_results.m           # 应用校准参数
└── imu_calibration_params.m # 校准参数存储
```

## 校准原理

### 加速度计误差模型

```
a_meas = T_a * diag(S_a) * a_true + b_a
```

其中：
- `b_a`：零偏 (Bias)
- `S_a`：尺度因子 (Scale Factor)
- `T_a`：失准角矩阵 (Misalignment)

校准目标：静态时加速度模长等于当地重力加速度 `g`

### 陀螺仪误差模型

```
ω_meas = T_g * diag(S_g) * ω_true + b_g
```

校准方法：利用相邻静态区间之间的重力矢量变化，与陀螺仪积分旋转进行比较

## 使用方法

### 1. 数据准备

将原始 IMU 数据放置在对应目录，运行数据预处理：

```matlab
data_pre
```

### 2. 执行校准

运行主校准脚本：

```matlab
cali_main
```

校准流程：
1. 加载数据与预处理
2. 静态区间检测
3. 加速度计参数优化
4. 陀螺仪参数优化
5. Allan 方差噪声分析
6. 结果验证

### 3. 应用校准参数

```matlab
cali_results
```

## 数据格式

输入数据要求：
- 采样率：100 Hz
- 数据列：时间戳 + 加速度(x,y,z) + 角速度(x,y,z)
- 加速度单位：g（程序内部转换为 m/s²）
- 角速度单位：deg/s（程序内部转换为 rad/s）

## 校准参数说明

校准结果存储在 `imuData` 结构体中：

| 参数 | 说明 | 单位 |
|------|------|------|
| `ab` | 加速度计零偏 | m/s² |
| `aK` | 加速度计尺度因子 | - |
| `aT` | 加速度计失准角矩阵 | rad |
| `gb` | 陀螺仪零偏 | rad/s |
| `gK` | 陀螺仪尺度因子 | - |
| `gT` | 陀螺仪失准角矩阵 | rad |

## 数据校正公式

```matlab
% 加速度计校正: a_true = inv(T_a * diag(S_a)) * (a_meas - b_a)
acc_calib = (acc_raw - acc_bias) * M_acc';

% 陀螺仪校正: ω_true = inv(T_g * diag(S_g)) * (ω_meas - b_g)
gyro_calib = (gyro_raw - gyro_bias) * M_gyro';

% 其中 M = inv(T * diag(K))
```

## 依赖

- MATLAB R2019b 或更高版本
- Optimization Toolbox（用于 `lsqnonlin`）
- Navigation Toolbox（用于 `allanvar`、`quaternion`）

## 注意事项

1. 校准数据需要包含多个静态姿态（建议使用多面体标定法）
2. 每个静态区间应保持稳定至少 1 秒
3. 当地重力加速度需根据实际位置调整（默认 9.79362 m/s²）

## 验证指标

- 加速度计模长 RMSE（校准前后对比）
- 陀螺仪单次翻转姿态误差（度）
