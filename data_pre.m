clc;clear;

% 读取两个CSV文件
data1 = readtable("0210_1\IMU_Data_20260210_1235.csv");
data2 = readtable("0210_1\IMU_Data_20260210_1246_part1.csv");
% 按行拼接数据
IMUdata1 = [data1; data2];
% 保存为.mat文件
save("IMUdata1.mat", "IMUdata1");

data3 = readtable("0210_2\IMU_Data_20260210_1251.csv");
data4 = readtable("0210_2\IMU_Data_20260210_1302_part1.csv");
% 按行拼接数据
IMUdata2 = [data3; data4];
% 保存为.mat文件
save("IMUdata2.mat", "IMUdata2");


