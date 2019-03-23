function [ time_sys, acc, gyro, mag ] = interoplate( acc_raw, gyro_raw, mag_raw, sf )
%INTERPLOT Summary of this function goes here
%   Detailed explanation goes here
% 原始数据格式为4列，时间戳，X轴，Y轴，Z轴
% 输出数据无时间戳
time_acc = acc_raw(:,1)/1e9;
time_gyro = gyro_raw(:,1)/1e9;
time_mag = mag_raw(:,1)/1e9;
%处理数据，去除时间重复点
%%
acc_raw_new=acc_raw;
gyro_raw_new=gyro_raw;
mag_raw_new=mag_raw;
for i=length(time_acc):-1:2
    if time_acc(i-1)==time_acc(i)
        acc_raw_new(i-1,:)=[];
    end
end
for j=length(time_gyro):-1:2
    if time_gyro(j-1)==time_gyro(j) 
        gyro_raw_new(j-1,:)=[];
    end
end
for k=length(time_mag):-1:2
    if time_mag(k-1)==time_mag(k)
    mag_raw_new(k-1,:)=[];
    end
end
time_acc_new = acc_raw_new(:,1)/1e9;
time_gyro_new = gyro_raw_new(:,1)/1e9;
time_mag_new = mag_raw_new(:,1)/1e9;

        
%time_start = max([time_acc_new(1), time_gyro_new(1), time_mag_new(1)]);
%time_end = min([time_acc_new(end), time_gyro_new(end), time_mag_new(end)]);
time_start = ceil(max([time_acc_new(1), time_gyro_new(1)]));
time_end =floor( min([time_acc_new(end), time_gyro_new(end)]));
time_sys = time_start:1/sf:time_end;
%%
acc =	interp1(time_acc_new, acc_raw_new(:,2:4), time_sys, 'spline'); %已知，time_acc与acc的点集，在time_sys上三次插值平滑结果
gyro = interp1(time_gyro_new, gyro_raw_new(:,2:4), time_sys, 'spline');
mag = interp1(time_mag_new, mag_raw_new(:,2:4), time_sys, 'spline');
%mag=mag_raw_new;   
end

