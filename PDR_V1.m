%% get data from different sensors
% acc->g;   gyro-> rad/s; mag-> uT
% clear all;


headingMethod = 'Yaw'; % Yaw, PCA, Acc,fused
sensor_type = 'IPhone';%'SBG'.'IPhone'

%输出的GNSS修正后的PDR的GPGGA信息路径
logFilePath       = '.\logfile\';
logNamePart = 'PDR_30';
logNameUse = strcat(logNamePart, '_', 'kalman');
fusion_tag  = 1 ; % 0代表只有PDR的GPGGA,1代表PDR+GNSS的GPGGA

%  filename = 'E:\导航组\PDR相关\pdr\data\309m\309m_2018816';  
filename = 'E:\导航组\测试数据\2018年11月29号 PDR+GNSS (马路+隧道绕环+宿舍区绕环）\隧道回环\2018_11_28_22_56_34_';
filename_gnss = 'E:\导航组\测试数据\2018年11月29号 PDR+GNSS (马路+隧道绕环+宿舍区绕环）\定位结果\NLS_IMU融合-4_GNSS_PDR.txt';
[timestamp_gnss,px_gnss,py_gnss,pz_gnss,vx_gnss,vy_gnss,vz_gnss] = textread(filename_gnss,'%f %f %f %f %f %f %f','delimiter',',');
 %% test file for smartphone with bias
 
	if strcmp(sensor_type,'IPhone')
    sampleFreq = 50; % 这个频率应该写个代码判断一下的,而且因为手机频率不稳定，可以适当调低这个频率设置

    acc_filename = [filename,'acc','.txt'];
    gyro_filename =[filename,'gyro','.txt'];
    mag_filename =[filename,'mag','.txt'];

    [ time_pdr,timestamp, acc, gyro, mag] = getDataFromLGphone( acc_filename, gyro_filename, mag_filename, 1/sampleFreq);
    

    magfile=load(mag_filename);
    magdata=magfile(:,4);
end
[ nameAbbr, data_location, data_use_mode ] = get_file_info( filename );
nameAbbr='ldh';
data_use_mode='compass';

%% detect step
do_plot = true;
[ acc_filtered ] = acc_filter( acc',sampleFreq,3 );
[ mag_filtered ] = acc_filter( mag',sampleFreq,3 );
update_points = StepCounter(acc_filtered, sampleFreq, do_plot);

%% calculate stride length
fitMode = 1;
[stride_length_record] = StrideEstimation (nameAbbr, fitMode, sampleFreq, acc, update_points);

%% calculate heading
%这个方法以西为0度，以南方向为正向（yawbias做了补偿），虽然是六轴，但是初始航向依然使用磁力计确定的。
step_counter = nnz(update_points);
heading = zeros(step_counter+1,2);

AHRS_mode = '6DOF_only';    % 9DOF_only, 6DOF_only, fused(default)
[ AHRS_euler,AHRS_quat,mag_test ] = AHRS_enhanced( sensor_type, gyro, acc, mag,1/sampleFreq,1,0.2,AHRS_mode);%1,0.2

mag_true_value = [48.5,33.47,46.383 ]; 

[ duruation] = stillDetect( AHRS_euler(:,3),(sum(gyro'.^2)).^0.5', 1 );%返回认为是静止状态的起始点和终点
%% 
local_magn_bias = -5.5;  %本地磁偏角
if strcmp(headingMethod,'Yaw')    
    [ headingRecord ] = headingFromYaw( acc_filtered, AHRS_euler(:,3) );
    if strcmp(data_use_mode,'compass')
        heading_initial = 0;
        yawHandDown = 0;
    else
        heading_initial = mean(AHRS_euler(duruation(1):duruation(2),3));
        yawHandDown = median(headingRecord(1:10,2));
    end
     % axis is point to me not the facing!!!
    if strcmp(sensor_type, 'IPhone') %&& strcmp(data_use_mode, 'compass')
        yawbias = heading_initial-yawHandDown+local_magn_bias-180;%原来这里local_magn_bias为加号
    else
        yawbias = heading_initial-yawHandDown+180+local_magn_bias;
    end
end

match_idx = knnsearch(headingRecord(:,1), update_points(1:step_counter)','K',1);
headingRecord2 = [update_points', AHRS_euler(update_points,3)];

%% calculate location
location_update = zeros(step_counter,3);
vel_pdr = zeros(step_counter,3);
vel_pdr(1,:)=[0,0,0];
%起始点坐标设定
floor = 3;
% location_update(1,:) = [23.3,4,0];    %实验室坐标
location_update(1,:) =[px_gnss(1),py_gnss(1),pz_gnss(1)];%GNSS初始坐标
location_update(:,3) = pz_gnss(1);
% map_bais = 22*pi/180;
map_bais = 0;
% map_bais = 54*pi/180;
ref_enu = [];
ref_enu_step=zeros(step_counter,2);
if strcmp(data_location,'outdoor') || strcmp(data_location,'nanti')
    [ ref_enu ] = double(llh2enu( LLH_IMU ));
end


%% PDR+GNSS KALMAN filter
% vel_pdr() PDR的速度 location_update PDR的位置  heading PDR的航向  time_stride_heading(i,4)时间戳
% px_gnss,py_gnss,vx_gnss,vy_gnss,）
X = zeros(length( location_update(:,1) ),5) ; %状态量：1、3代表XY方向的deltaP，2、4对应delta_v,5代表fai
X_p = zeros(length(location_update(:,1) ),5) ; %状态量的预测值
Y = zeros(length(location_update(:,1) ),5) ; %观测量
X(1,:) = Y(1,:);
Q = diag([1 0.1 1 0.1 0.2]); %这个参数需要特别修正
R = diag([1 0.1 1 0.1 0.2]);  %这个参数也需要特别修正
P0=diag([1 1 1 1 1 ]); 
P = P0;
P_p =zeros(5,5); %协方差的预测值


time_stride_heading = zeros((step_counter),4); %储存每个点的时间、步长和航向信息
delta_time_pdr =  zeros(step_counter+1,1);
delta_time_pdr(1) = timestamp(update_points(1)) - timestamp(1);
time_step  = zeros(length(location_update(:,1) ),1);
time_step(1) = time_pdr(1);

fai_gnss = zeros(length(px_gnss),1);
fai_gnssv = zeros(length(px_gnss),1);
for i=2:length(px_gnss) 
    fai_gnss(i) = atan2( (py_gnss(i)-py_gnss(i-1)),(px_gnss(i)-px_gnss(i-1)) );  %基于位置的求GNSS航向
    fai_gnssv(i)  =( atan2(vy_gnss(i),vx_gnss(i)) )  ; %基于速度的求GNSS航向
end
fai_gnss = fai_gnssv;

for i= 1:step_counter
    if i == length(update_points)
       break;
    end
    heading(i+1,2) = headingRecord(match_idx(i),2)+yawbias; 
    if  heading(i+1,2) >180 %把角度限定到-pi~pi
        heading(i+1,2) = heading(i+1,2) -360;
    elseif heading(i+1,2) < -200
        heading(i+1,2) = heading(i+1,2) +360;
    end
    heading(i+1,2) = heading(i+1,2)*pi/180; %航向，弧度表示
    time_stride_heading(i,1) = update_points(i);
    time_stride_heading(i,2) = stride_length_record(i);
    time_stride_heading(i,3) = heading(i+1,2);   
    time_stride_heading(i,4) = timestamp(update_points(i));
    time_step(i+1)  = time_pdr(update_points(i));
end

for i=1:step_counter-1

    location_update(i+1,1) = location_update(i,1) + time_stride_heading(i,2) * (cos(time_stride_heading(i,3)+map_bais));%经过这样的步骤已经把yaw变成了东北天坐标系，
    location_update(i+1,2) = location_update(i,2) + time_stride_heading(i,2) * (sin(time_stride_heading(i,3)+map_bais));
    vel_pdr(i+1,1) = ( time_stride_heading(i,2) * (cos(time_stride_heading(i,3)+map_bais))  )  / ( timestamp(update_points(i+1)) -timestamp(update_points(i)) ) ; 
    vel_pdr(i+1,2) = ( time_stride_heading(i,2) * (sin(time_stride_heading(i,3)+map_bais))  )  / ( timestamp(update_points(i+1)) -timestamp(update_points(i)) ) ; 
    
    for   j  = 1: length(timestamp_gnss)  %寻找PDR某一步对应的时间点
       if ( time_step(i+1) - timestamp_gnss(j) ) ==0
            Y(i+1,1) = location_update(i+1,1) - px_gnss(j);
            Y(i+1,3) = location_update(i+1,2) - py_gnss(j);
            location_update(i,3)   =  pz_gnss(j);
            Y(i+1,2) = vel_pdr(i+1,1) - vx_gnss(j);
            Y(i+1,4) = vel_pdr(i+1,2) - vy_gnss(j);
            Y(i+1,5) = heading(i+1,2) - fai_gnss(j);
       end
    end

   delta_t = ( timestamp(update_points(i+1)) -timestamp(update_points(i)) );
    t = delta_t;
    A=[1 t 0 0 0 ;0 1 0 0 0;0 0 1 t 0 ;0 0 0 1 0 ;0 0 0 0 1];
    X_p(i+1,1) =  X(i,1)  + X(i,2)*t;
    X_p(i+1,2) =  X(i,2);    
    X_p(i+1,3) =  X(i,3)  + X(i,4)*t;
    X_p(i+1,4) =  X(i,4);    
    X_p(i+1,5) =  X(i,5);
    P_p= A*P*A' +Q;
    
    Kr = P_p/(P_p+R);
    X_p_ = [ X_p(i+1,1) ; X_p(i+1,2) ; X_p(i+1,3) ; X_p(i+1,4) ; X_p(i+1,5) ;];
    dx  = [ ( Y( (i+1) ,1) - X_p(i+1,1) ); ( Y( (i+1) ,2) - X_p(i+1,2) ); ( Y( (i+1) ,3) - X_p(i+1,3) ); ( Y( (i+1) ,4) - X_p(i+1,4) ); ( Y( (i+1) ,5) - X_p(i+1,5) ); ];
    X(i+1,:) = ( X_p_ + Kr * dx )';
    P = (eye(5) - Kr )*P_p;
    if fusion_tag == 1
        location_update(i+1,1) = location_update(i+1,1) -  0.05*X(i+1,1);
        location_update(i+1,2) = location_update(i+1,2) - 0.05*X(i+1,3);
    elseif fusion_tag==0
        location_update(i+1,1) = location_update(i+1,1);
        location_update(i+1,2) = location_update(i+1,2);
    end
    vel_pdr(i+1,1)  = vel_pdr(i+1,1) - X(i+1,2);
    vel_pdr(i+1,2)  = vel_pdr(i+1,2) - X(i+1,4);
    heading(i+1,2) = heading(i+1,2) - X(i+1,5);
   
end

%% plot track

plot_route( location_update );
title(['Route of  ZHP' ,' using ', sensor_type, ' in mode of ', data_use_mode,' @ ',data_location,', ',headingMethod,' heading']);


% figure
plot_route( location_update );
title('calibration');
%%
%输出GPGGA文件
Q1=[-2853702.49460991,4667244.62864575,3268344.29550058];%马路东点
Q2=[-2853298.00149200,4667616.63894344,3268172.28716137];%马路西点

Q2_LLH= zeros(3,1);
[Q2_LLH(1),Q2_LLH(2),Q2_LLH(3)] = cart2geo(Q2(1),Q2(2),Q2(3),5);

Q1_ENU=zeros(1,3);
Q2_ENU=zeros(1,3);
[Q2_ENU(1),Q2_ENU(2),Q2_ENU(3)] =ecef2enu(Q2(1),Q2(2),Q2(3),Q2_LLH(1),Q2_LLH(2),Q2_LLH(3),wgs84Ellipsoid,'degrees');
[Q1_ENU(1),Q1_ENU(2),Q1_ENU(3)] =ecef2enu(Q1(1),Q1(2),Q1(3),Q2_LLH(1),Q2_LLH(2),Q2_LLH(3),wgs84Ellipsoid,'degrees');

LLH= zeros(3,1);

time = struct('hour',   [], 'min',  [],'sec',   []);

for i = 1:length(location_update(:,1))-1
    ENUXYZ = [location_update(i,1), location_update(i,2), location_update(i,3)];
    k = (Q1_ENU(2)-Q2_ENU(2))/(Q1_ENU(1)-Q2_ENU(1));
    b = Q1_ENU(2) - k*Q1_ENU(1);
    error_ENU(i) = ENUXYZ(2) - (k*ENUXYZ(1)+b);   
%     ENUXYZ = [px_gnss(i),py_gnss(i),pz_gnss(i)];
    WGS84XYZ = zeros(3,1);
    [WGS84XYZ(1),WGS84XYZ(2),WGS84XYZ(3)] = enu2ecef(ENUXYZ(1),ENUXYZ(2),ENUXYZ(3),Q2_LLH(1),Q2_LLH(2),Q2_LLH(3),wgs84Ellipsoid,'degrees');
    [LLH(1),LLH(2),LLH(3)]= cart2geo(WGS84XYZ(1),WGS84XYZ(2),WGS84XYZ(3),5);
    time_temp=num2str(time_step(i));    
    time.hour = str2double(time_temp(9:10));
    time.min = str2double(time_temp(11:12));
    time.sec = str2double(time_temp(13:14));
    OutputGPGGA(LLH(1), LLH(2), LLH(3), time, 0, logFilePath, logNameUse,1)
end

