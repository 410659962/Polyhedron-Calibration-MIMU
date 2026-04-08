clc;clear;


data1 = readtable("0210_1\IMU_Data_20260210_1235.csv");
data2 = readtable("0210_1\IMU_Data_20260210_1246_part1.csv");

IMUdata1 = [data1; data2];

save("IMUdata1.mat", "IMUdata1");

data3 = readtable("0210_2\IMU_Data_20260210_1251.csv");
data4 = readtable("0210_2\IMU_Data_20260210_1302_part1.csv");

IMUdata2 = [data3; data4];

save("IMUdata2.mat", "IMUdata2");


