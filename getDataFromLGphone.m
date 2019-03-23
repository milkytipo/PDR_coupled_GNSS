function [ time_pdr,timestamp,acc,gyro,mag] = getDataFromLGphone( acc_filename, gyro_filename, mag_filename, dT )

%% get raw data
%处理方法：每三个点取一个，然后通过插值函数interp1进行填补
[time_pdr,timestamp,acc_raw,gyro_raw,mag_raw ] = getRawDataFromLGphone( acc_filename, gyro_filename, mag_filename, dT );


acc = acc_raw;
gyro = gyro_raw;
mag = mag_raw;

%% calibration
% load('test_data\LGphone_data\LGcalibration.mat');
% [ mag ]  = calibrate_data( magn_coefs, mag_raw );
% [ acc ]  = calibrate_data( acc_coefs, acc_raw );
% [ gyro ] = calibrate_data( gyro_coefs, gyro_raw );

end