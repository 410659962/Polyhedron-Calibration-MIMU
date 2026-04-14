# IMUArray Fusion IMU Calibration

## 项目简介

本项目实现了一套完整的MEMS IMU（惯性测量单元）标定系统，采用12参数模型对加速度计和陀螺仪进行高精度标定。标定方法利用静态区间检测与Levenberg-Marquardt优化算法，无需高精度转台，适用于IMU阵列的现场标定。

## 标定模型

### 12参数标定模型

传感器测量模型：
```
a_meas = T^a · K^a · a_true + b^a
ω_meas = T^g · K^g · ω_true + b^g
```

校准反演公式：
```
a_true = (a_meas - b^a) / (T^a · K^a)'
ω_true = (ω_meas - b^g) / (T^g · K^g)'
```

参数说明：
| 参数 | 维度 | 含义 |
|------|------|------|
| b (零偏) | 3×1 | 传感器零偏 |
| K (标度因子) | 3×1 | 三轴标度因子 |
| T (安装误差矩阵) | 3×3 | 非正交误差矩阵 |

安装误差矩阵结构：
```
T = |  1    -m1   m2  |
    | m3     1   -m4  |
    |-m5    m6    1  |
```

## 文件结构

```
IMU_calibration/
├── cali_main.m              # 主程序入口
├── detectStaticRegions.m    # 静态区间检测
├── acc_cost_function.m     # 加速度计代价函数
├── gyro_cost_function.m     # 陀螺仪代价函数
├── cali_fig.m               # 结果可视化
├── data_pre.m               # CSV数据预处理
├── imu_calibration_params.m # 标定参数结构体定义
├── IMUdata1.mat             # 标定数据1
├── IMUdata2.mat             # 标定数据2
├── 0210_1/                  # 原始CSV数据
├── 0210_2/                  # 原始CSV数据
└── cali_results/            # 标定结果输出
```

## 核心算法流程

### 1. 数据加载与预处理
```matlab
load("IMUdata1.mat"); load("IMUdata2.mat");
imu_raw = [IMUdata1{:,1}, IMUdata1{:,26:31}];
acc_raw = imu_raw(:, 2:4) * 9.79362;   % 转换为m/s²
gyro_raw = imu_raw(:, 5:7) * pi/180;   % 转换为rad/s
fs = 100;                              % 采样频率100Hz
```

### 2. 静态区间检测
- 滑动窗口方差检测（窗口5秒）
- 加速度方差阈值：< 0.008
- 陀螺仪方差阈值：< 0.0005
- 最小静态时长：≥30秒（适配14面采集）
- 智能合并：间隔<2秒且方向变化<5°的区间合并

### 3. 加速度计标定
- 优化变量：12参数 [bx, by, bz, sx, sy, sz, m1~m6]
- 代价函数：静态区间加速度模长与重力参考值的残差
- 优化算法：Levenberg-Marquardt（FunctionTolerance: 1e-9）

### 4. 陀螺仪标定
- 优化变量：12参数 [bx, by, bz, sx, sy, sz, m1~m6]
- 零偏初始值：静态区间陀螺仪均值
- 代价函数：重力矢量积分残差
  - 四元数梯形积分更新姿态
  - 预测下一静态点重力方向
  - 计算与实际重力观测值的残差

### 5. 结果验证
- 加速度计：静态区间模长RMSE
- 陀螺仪：重力矢量残差模长RMSE

## 使用方法

### 数据预处理
```matlab
>> data_pre
```
将CSV原始数据拼接转换为MAT格式。

### 运行标定
```matlab
>> cali_main
```

### 查看标定参数
```matlab
>> imu_calibration_params
```

## 输出结果

### 标定参数
- 加速度计零偏 b^a (m/s²)
- 加速度计标度因子 K^a
- 加速度计安装误差矩阵 T^a
- 陀螺仪零偏 b^g (rad/s)
- 陀螺仪标度因子 K^g
- 陀螺仪安装误差矩阵 T^g

### 可视化图表
1. 静态检测与加速度数据
2. 加速度计X/Y/Z轴标定前后对比
3. 加速度计模长对比
4. 陀螺仪X/Y/Z轴标定前后对比

### 验证指标
```
加速度计模长 RMSE (标定前 -> 标定后): xxx -> xxx m/s²
陀螺仪重力矢量残差模长 RMSE (标定前 -> 标定后): xxx -> xxx m/s²
```

## 算法特点

1. **无需转台**：利用地球重力场作为参考，通过多位置静态采集实现标定
2. **12参数全优化**：零偏、标度因子、安装误差矩阵联合优化
3. **严格正演模型**：采用一致的校准反演公式，避免误差传播
4. **智能静态检测**：多指标融合与方向约束，确保静态判别准确
5. **高精度优化**：Levenberg-Marquardt算法配合严密收敛准则

---

## 多MIMU校准

### 概述

`multi_cali_main.m` 提供了一次性校准多个MIMU（最多9个）的功能，适用于IMU阵列的批量标定场景。该程序复用单MIMU标定的核心算法，通过循环处理实现高效批量校准。

### 数据格式

输入CSV文件格式（`IMUdata1.csv`）：
| 列号 | 数据内容 | 单位 |
|------|----------|------|
| 1 | 时间戳 | s |
| 2-4 | MIMU1 加速度计 (x,y,z) | g |
| 5-7 | MIMU1 陀螺仪 (x,y,z) | deg/s |
| 8-10 | MIMU2 加速度计 (x,y,z) | g |
| 11-13 | MIMU2 陀螺仪 (x,y,z) | deg/s |
| ... | ... | ... |
| 50-52 | MIMU9 加速度计 (x,y,z) | g |
| 53-55 | MIMU9 陀螺仪 (x,y,z) | deg/s |

### 使用方法

```matlab
>> multi_cali_main
```

程序将自动：
1. 加载所有9个MIMU的原始数据
2. 使用MIMU1的数据检测静态区间（所有MIMU共用）
3. 依次对每个MIMU进行加速度计和陀螺仪标定
4. 输出验证结果和校准后数据

### 输出文件

#### 1. multi_output.csv
校准后的所有MIMU数据，格式如下：
| 列号 | 数据内容 | 单位 |
|------|----------|------|
| 1 | 时间戳 | s |
| 2-4 | MIMU1 校准后加速度计 (x,y,z) | m/s² |
| 5-7 | MIMU1 校准后陀螺仪 (x,y,z) | rad/s |
| 8-10 | MIMU2 校准后加速度计 (x,y,z) | m/s² |
| 11-13 | MIMU2 校准后陀螺仪 (x,y,z) | rad/s |
| ... | ... | ... |

#### 2. calib_params.mat
包含所有MIMU标定参数的结构体数组：
```matlab
calib_params(i).acc_b      % 第i个MIMU的加速度计零偏
calib_params(i).acc_s      % 第i个MIMU的加速度计标度因数
calib_params(i).acc_T      % 第i个MIMU的加速度计安装误差矩阵
calib_params(i).gyro_b     % 第i个MIMU的陀螺仪零偏
calib_params(i).gyro_s     % 第i个MIMU的陀螺仪标度因数
calib_params(i).gyro_T     % 第i个MIMU的陀螺仪安装误差矩阵
calib_params(i).acc_calib  % 第i个MIMU校准后的加速度计数据
calib_params(i).gyro_calib % 第i个MIMU校准后的陀螺仪数据
```

### 验证输出示例

```
MIMU     | Acc RMSE (前->后)     | Gyro RMSE (前->后)
------------------------------------------------------------
MIMU 1   | 0.12345 -> 0.01234   | 0.001234 -> 0.000123
MIMU 2   | 0.13456 -> 0.01345   | 0.001345 -> 0.000134
...
MIMU 9   | 0.14567 -> 0.01456   | 0.001456 -> 0.000145
```

### 算法特点

1. **批量处理**：一次运行完成所有MIMU的标定，提高效率
2. **共用静态区间**：所有MIMU使用相同的静态区间检测结果，确保一致性
3. **独立优化**：每个MIMU独立进行12参数优化，互不干扰
4. **统一输出**：校准后数据和参数集中存储，便于后续使用
