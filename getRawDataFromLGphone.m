function [ time_pdr_new,timestamp,acc,gyro,mag ] = getRawDataFromLGphone( acc_filename, gyro_filename, mag_filename, T )
%% get data

acc_data_array = importdata(acc_filename);
gyro_data_array = importdata(gyro_filename);
acc_data = acc_data_array.data;
acc_data(:,2:4) = acc_data(:,2:4)/9.8;
mag_data_raw = importdata(mag_filename);

gyro_data = gyro_data_array.data;
mag_data = mag_data_raw.data;
for i =1:length(acc_data)
    acc_temp(ceil(i/3),:)=acc_data(i,:);
end
for i =1:length(gyro_data)
    gyro_temp(ceil(i/3),:)=gyro_data(i,:);
end
for i =1:length(mag_data)
    mag_temp(ceil(i/3),:)=mag_data(i,:);
end

acc_data=acc_temp;
mag_data=mag_temp;
gyro_data=gyro_temp;
 if T > 0 
    %interplot
    sf = 1/T; %sf = 100
    [ timestamp, acc, gyro, mag ] = interoplate( acc_data, gyro_data, mag_data, sf );
 else
    timestamp=acc_data(:,1);
    acc  = acc_data(:,2:4);
    gyro = gyro_data(:,2:4);
    mag  = mag_data(:,2:4);
    %timestamp = 0;
 end
 
 %pdr的Date格式时间戳
time_pdr =zeros(length(acc_data_array.data(:,1)),1);
for i = 1:length(acc_data_array.data(:,1))
    time_pdr(i) =int64(str2double(  strcat( acc_data_array.textdata{i,1}(1:4),acc_data_array.textdata{i,1}(6:7),acc_data_array.textdata{i,1}(9:10),...
        acc_data_array.textdata{i,1}(14:15),acc_data_array.textdata{i,1}(17:18),acc_data_array.textdata{i,1}(20:21))   ));
end
time_pdr_start  = time_pdr(1);
time_pdr_end = time_pdr(end);
time_pdr_new = zeros(length(timestamp),1);
for  i =1:(length(timestamp)/sf )
    for j  =1:sf
        time_pdr_new((i-1)*sf +j ) = time_pdr( floor(    (i-1)*sf*(length(time_pdr)/length(timestamp))+1  )  ); %floor(time_pdr/timestamp)因为频率不稳定
    end
end
for j  =(i)*sf : length(time_pdr_new)
    time_pdr_new( j ) = time_pdr(floor(    (i-1)*sf*(length(time_pdr)/length(timestamp))+1  ));
end
% end

