clc;
clear;
load("merge_data.mat");
load("imu_calibration_params.mat");

imu_raw1 = [merge_data{:,1}, merge_data{:,2:7}];

t1 = imu_raw1(:, 1);
acc_raw1 = imu_raw1(:, 2:4) * 9.79362;%g->m/s^2
gyro_raw1 = imu_raw1(:, 5:7) * pi/180;%deg/s->rad/s
acc_calib1 = (acc_raw1 - imuData(1).ab) * imuData(1).M_acc';
gyro_calib1 = (gyro_raw1 - imuData(1).gb) * imuData(1).M_gyro';
imucali1 = [t1, acc_calib1, gyro_calib1];
save('cali_results/imucali1.csv', 'imucali1');

imu_raw2 = [merge_data{:,1}, merge_data{:,8:13}];

t2 = imu_raw2(:, 1);
acc_raw2 = imu_raw2(:, 2:4) * 9.79362;%g->m/s^2
gyro_raw2 = imu_raw2(:, 5:7) * pi/180;%deg/s->rad/s
acc_calib2 = (acc_raw2 - imuData(2).ab) * imuData(2).M_acc';
gyro_calib2 = (gyro_raw2 - imuData(2).gb) * imuData(2).M_gyro';
imucali2 = [t2, acc_calib2, gyro_calib2];
save('cali_results/imucali2.mat', 'imucali2');

imu_raw3 = [merge_data{:,1}, merge_data{:,14:19}];

t3 = imu_raw3(:, 1);
acc_raw3 = imu_raw3(:, 2:4) * 9.79362;%g->m/s^2
gyro_raw3 = imu_raw3(:, 5:7) * pi/180;%deg/s->rad/s
acc_calib3 = (acc_raw3 - imuData(3).ab) * imuData(3).M_acc';
gyro_calib3 = (gyro_raw3 - imuData(3).gb) * imuData(3).M_gyro';
imucali3 = [t3, acc_calib3, gyro_calib3];
save('cali_results/imucali3.mat', 'imucali3');

imu_raw4 = [merge_data{:,1}, merge_data{:,20:25}];

t4 = imu_raw4(:, 1);
acc_raw4 = imu_raw4(:, 2:4) * 9.79362;%g->m/s^2
gyro_raw4 = imu_raw4(:, 5:7) * pi/180;%deg/s->rad/s
acc_calib4 = (acc_raw4 - imuData(4).ab) * imuData(4).M_acc';
gyro_calib4 = (gyro_raw4 - imuData(4).gb) * imuData(4).M_gyro';
imucali4 = [t4, acc_calib4, gyro_calib4];
save('cali_results/imucali4.mat', 'imucali4');

imu_raw5 = [merge_data{:,1}, merge_data{:,26:31}];

t5 = imu_raw5(:, 1);
acc_raw5 = imu_raw5(:, 2:4) * 9.79362;%g->m/s^2
gyro_raw5 = imu_raw5(:, 5:7) * pi/180;%deg/s->rad/s
acc_calib5 = (acc_raw5 - imuData(5).ab) * imuData(5).M_acc';
gyro_calib5 = (gyro_raw5 - imuData(5).gb) * imuData(5).M_gyro';
imucali5 = [t5, acc_calib5, gyro_calib5];
save('cali_results/imucali5.mat', 'imucali5');

imu_raw6 = [merge_data{:,1}, merge_data{:,32:37}];

t6 = imu_raw6(:, 1);
acc_raw6 = imu_raw6(:, 2:4) * 9.79362;%g->m/s^2
gyro_raw6 = imu_raw6(:, 5:7) * pi/180;%deg/s->rad/s
acc_calib6 = (acc_raw6 - imuData(6).ab) * imuData(6).M_acc';
gyro_calib6 = (gyro_raw6 - imuData(6).gb) * imuData(6).M_gyro';
imucali6 = [t6, acc_calib6, gyro_calib6];
save('cali_results/imucali6.mat', 'imucali6');

imu_raw7 = [merge_data{:,1}, merge_data{:,38:43}];

t7 = imu_raw7(:, 1);
acc_raw7 = imu_raw7(:, 2:4) * 9.79362;%g->m/s^2
gyro_raw7 = imu_raw7(:, 5:7) * pi/180;%deg/s->rad/s
acc_calib7 = (acc_raw7 - imuData(7).ab) * imuData(7).M_acc';
gyro_calib7 = (gyro_raw7 - imuData(7).gb) * imuData(7).M_gyro';
imucali7 = [t7, acc_calib7, gyro_calib7];
save('cali_results/imucali7.mat', 'imucali7');

imu_raw8 = [merge_data{:,1}, merge_data{:,44:49}];

t8 = imu_raw8(:, 1);
acc_raw8 = imu_raw8(:, 2:4) * 9.79362;%g->m/s^2
gyro_raw8 = imu_raw8(:, 5:7) * pi/180;%deg/s->rad/s
acc_calib8 = (acc_raw8 - imuData(8).ab) * imuData(8).M_acc';
gyro_calib8 = (gyro_raw8 - imuData(8).gb) * imuData(8).M_gyro';
imucali8 = [t8, acc_calib8, gyro_calib8];
save('cali_results/imucali8.mat', 'imucali8');

imu_raw9 = [merge_data{:,1}, merge_data{:,50:55}];

t9 = imu_raw9(:, 1);
acc_raw9 = imu_raw9(:, 2:4) * 9.79362;%g->m/s^2
gyro_raw9 = imu_raw9(:, 5:7) * pi/180;%deg/s->rad/s
acc_calib9 = (acc_raw9 - imuData(9).ab) * imuData(9).M_acc';
gyro_calib9 = (gyro_raw9 - imuData(9).gb) * imuData(9).M_gyro';
imucali9 = [t9, acc_calib9, gyro_calib9];
save('cali_results/imucali9.mat', 'imucali9');

merge_cali = [t1, acc_calib1, gyro_calib1, acc_calib2, gyro_calib2, acc_calib3, gyro_calib3, ...
              acc_calib4, gyro_calib4, acc_calib5, gyro_calib5, acc_calib6, gyro_calib6, ...
              acc_calib7, gyro_calib7, acc_calib8, gyro_calib8, acc_calib9, gyro_calib9];
save('cali_results/merge_cali.mat', 'merge_cali');
