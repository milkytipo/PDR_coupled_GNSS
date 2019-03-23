function [ AHRS_euler,AHRS_quaternion,mag_test ] = AHRS_enhanced( sensor_type, gyro, acc, mag,dT,stage1_factor, stage2_factor, mode_select )
% interval_second = 0.2;
% interval = round(interval_second/dT);
%% filter parm setting
switch nargin
    case 5
        stage1_factor = 0.7;      % 0.5~1
        stage2_factor = 0.2;   % 0.1~0.2
        mode_select = 'fused';
    case 6
        stage2_factor = 0.2;
        mode_select = 'fused';
    case 7
        mode_select = 'fused';
end
%% time setting
interval = 4;       % check whether magnetic yaw is good every 'interval' sampling points
compare_durance = 2*interval;               % compare_durance is Integer multiples of interval
init_time = round(2/dT/interval)*interval;  % init_time is Integer multiples of interval

%% inital 2 AHRSs
beta = 0.041;
beta_init = 2;
AHRS_9dof = MadgwickAHRS('SamplePeriod', dT, 'Beta', beta_init);
AHRS_6dof = MadgwickAHRS('SamplePeriod', dT, 'Beta', beta_init);
sample_cnt = length(acc(:,1));
AHRS_quaternion = zeros(sample_cnt, 4);
mag_test = zeros(sample_cnt, 3); 
% Total field; Horizontal Field; Vertical Field; Inclination Angle 
quaternion_9dof_buf = zeros(sample_cnt, 4);
quaternion_6dof_buf = zeros(sample_cnt, 4);
count_9dof = 0;
count_6dof = 0;
% Total field, Horizontal Field, Inclination Angle
true_value = [48.5,33.47,46.383 ];  % location related 
threshold_collection = [9.3,8,10.7]*stage1_factor;  %[9.3,8,10.7]
yaw_dif_threshold = compare_durance*stage2_factor;   %0.18

if strcmp(mode_select, '9DOF_only') 
    for t = 1:sample_cnt
        if t == init_time
            AHRS_9dof = MadgwickAHRS('SamplePeriod', dT, 'Beta', beta,'Quaternion',AHRS_9dof.Quaternion);
        end
        AHRS_9dof.Update(gyro(t,:) , acc(t,:), mag(t,:)); 
        if strcmp(sensor_type, 'IPhone')  ||strcmp(sensor_type, 'LGphone')
            AHRS_quaternion(t,:) = AHRS_9dof.Quaternion;
        elseif strcmp(sensor_type, 'SBG') || strcmp(sensor_type, 'SBG_interplot')
            AHRS_quaternion(t,:) = SBG_Q(AHRS_9dof.Quaternion);
        end
        count_9dof = count_9dof+1;
    end
elseif strcmp(mode_select, '6DOF_only')
    for t = 1:sample_cnt
        AHRS_6dof.UpdateIMU(gyro(t,:) , acc(t,:));
        if t < init_time 
            AHRS_9dof.Update(gyro(t,:) , acc(t,:), mag(t,:));
            AHRS_6dof.Quaternion = AHRS_9dof.Quaternion;
        end
        if t == init_time
            AHRS_6dof = MadgwickAHRS('SamplePeriod', dT, 'Beta', beta,'Quaternion',AHRS_9dof.Quaternion);
        end
        count_6dof = count_6dof+1;
        if strcmp(sensor_type, 'IPhone') ||strcmp(sensor_type, 'LGphone')
            AHRS_quaternion(t,:) = AHRS_6dof.Quaternion;
        elseif strcmp(sensor_type, 'SBG') || strcmp(mode_select, 'SBG_interplot')
            AHRS_quaternion(t,:) = SBG_Q(AHRS_6dof.Quaternion);
        end
    end
elseif strcmp(mode_select, 'fused')
    for t = 1:sample_cnt
        %% calculate 6DOF & 9DOF ahrs
        AHRS_9dof.Update(gyro(t,:) , acc(t,:), mag(t,:));
        AHRS_6dof.UpdateIMU(gyro(t,:) , acc(t,:));
        %% calculate magnetic related values
        if strcmp(sensor_type, 'IPhone') 
            mag_ENU = quaternRotate(mag(t,:),AHRS_6dof.Quaternion);
        elseif strcmp(sensor_type, 'SBG') || strcmp(mode_select, 'SBG_interplot')
            mag_ENU = quaternRotate(mag(t,:),SBG_Q(AHRS_6dof.Quaternion));
        end
        mag_test(t,1) = norm(mag(t,:));                                     % Total field
        mag_test(t,2) = norm([mag_ENU(1),mag_ENU(2)]);                      % Horizontal Field        
        mag_test(t,3) = atan2( mag_ENU(3),mag_test(t,2) )*180/pi;        % Inclination Angle 
        %% time related
        if t < init_time  % 5s
            if strcmp(sensor_type, 'IPhone') 
                AHRS_quaternion(t,:) = AHRS_9dof.Quaternion;
            elseif strcmp(sensor_type, 'SBG') || strcmp(mode_select, 'SBG_interplot')
                AHRS_quaternion(t,:) = SBG_Q(AHRS_9dof.Quaternion);
            end
            AHRS_6dof.Quaternion = AHRS_9dof.Quaternion;
            continue;
        elseif t <= init_time+compare_durance-interval
            if t == init_time    % init 6dof quat
                AHRS_9dof = MadgwickAHRS('SamplePeriod', dT, 'Beta', beta,'Quaternion',AHRS_9dof.Quaternion);
                AHRS_6dof = MadgwickAHRS('SamplePeriod', dT, 'Beta', beta,'Quaternion',AHRS_9dof.Quaternion);
            end
            if strcmp(sensor_type, 'IPhone') 
                AHRS_quaternion(t,:) = AHRS_9dof.Quaternion;
            elseif strcmp(sensor_type, 'SBG') || strcmp(mode_select, 'SBG_interplot')
                AHRS_quaternion(t,:) = SBG_Q(AHRS_9dof.Quaternion);
            end
        elseif mod(t,interval) == 0 && t >= init_time+compare_durance
        %% comparation
            yaw_difference = cal_yaw_diff(quaternion_6dof_buf(t-compare_durance+1:t,:),...
                                          quaternion_9dof_buf(t-compare_durance+1:t,:));
            % determin how many magentic point is good in the window of compare_durance
            mag_GoodOrNot2 = zeros(compare_durance,1);
            for k = 1:compare_durance
                check = abs(mag_test(t-k-1,:)-true_value)< threshold_collection;
                if (check(1) == 1 && check(2)==1)||(check(1) == 1 && check(3)==1)
                    mag_GoodOrNot2(k,1) = 1;
                end
            end
            if nnz(mag_GoodOrNot2) == compare_durance && yaw_difference<yaw_dif_threshold  % prefer 9_dof
                count_9dof = count_9dof+1;
                AHRS_6dof.Quaternion = AHRS_9dof.Quaternion;
                AHRS_quaternion(t-interval:t,:) = quaternion_9dof_buf(t-interval:t,:);
            else
                count_6dof = count_6dof+1;
                AHRS_9dof.Quaternion = AHRS_6dof.Quaternion;    % quat from 6DOF is the most accurate AHRS
                AHRS_quaternion(t-interval:t,:) = quaternion_6dof_buf(t-interval:t,:);
            end
        end
        if strcmp(sensor_type, 'IPhone') 
            quaternion_9dof_buf(t,:) = AHRS_9dof.Quaternion;
            quaternion_6dof_buf(t,:) = AHRS_6dof.Quaternion;
        elseif strcmp(sensor_type, 'SBG') || strcmp(mode_select, 'SBG_interplot')
            quaternion_9dof_buf(t,:) = SBG_Q(AHRS_9dof.Quaternion);
            quaternion_6dof_buf(t,:) = SBG_Q(AHRS_6dof.Quaternion);
        end

    end
end
AHRS_euler = quatern2euler(quaternConj(AHRS_quaternion))*180/pi;
ratio_9dof = count_9dof/(count_9dof+count_6dof);
disp(['Magnetic yaw is referenced ' num2str(ratio_9dof*100) '% of the total time.'])
end