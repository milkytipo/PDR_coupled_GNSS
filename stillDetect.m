function [ duruation] = stillDetect( yaw,gyro, start_index )
%	Detect still from the start index
%   Detailed explanation goes here
duruation = [];
% std_record = [];
T = 0.01;
miniStillPeriod = 1/T; %   1s of stillness is require
step = 1;   %miniStillPeriod/10;
gyro_threshold = 2;
yaw_threshold = 0.9;

%% find still start point 
while true
    k=start_index;
    while k <= length(yaw(:,1))-miniStillPeriod
        std_gyro = std(gyro(k:k+miniStillPeriod));%·µ»Ø±ê×¼²î
        std_yaw = std(yaw(k:k+miniStillPeriod));
%         std_record(end+1,1) = k;
%         std_record(end,2) = std_gyro;
%         std_record(end,3) = std_yaw;
        if std_gyro < gyro_threshold && std_yaw < yaw_threshold
            duruation(1,1) = k;
            break;
        end    
        k = k+step;
    end
    if isempty(duruation)
        gyro_threshold = 1.2*gyro_threshold;
        yaw_threshold = 1.2*yaw_threshold;
    else
        break;
    end
end
%% find still end point
k = k+miniStillPeriod;
while k <= length(yaw(:,1))
    std_gyro = std(gyro(k-miniStillPeriod:k));
    std_yaw = std(yaw(k-miniStillPeriod:k));
%     std_record(end+1,1) = k;
%     std_record(end,2) = std_gyro;
%     std_record(end,3) = std_yaw;
    if std_gyro > gyro_threshold || std_yaw > yaw_threshold
        duruation(1,2) = k;
        break;
    end    
    k = k+step;
end
end