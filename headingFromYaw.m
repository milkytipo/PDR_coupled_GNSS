function [ headingRecord ] = headingFromYaw( acc_filtered, AHRS_yaw )
% input: acc_filtered = n*1
%        AHRS_yaw     = n*1, the yaw from n*3 AHRS
% output: heading = n*2, [ timestamps[i], heading[i] ]
g = 9.8; %gravity
acc_filtered = acc_filtered/0.9135*g;
headingRecord = [];
for i=5:length(acc_filtered)-4
%     cof = polyfit(1:10,acc_m_FIR(i-5:i+4),1);
%     slopeRecord(end+1) = abs(cof(1));
    if acc_filtered(i-1) > g  && acc_filtered(i+1) < g
        if isempty(headingRecord)   % the first heading recorded
            cof = polyfit(1:9,acc_filtered(i-4:i+4),1);
            if abs(cof(1))>0.2 %0.3
                headingRecord(end+1,1) = i;
                headingRecord(end,2) = AHRS_yaw(i); 
            end
        elseif i-headingRecord(end,1)>3   % if two heading is too close they are acrutely one
            cof = polyfit(1:9,acc_filtered(i-4:i+4),1);
            if abs(cof(1))>0.2 %0.3
                headingRecord(end+1,1) = i;
                headingRecord(end,2) = AHRS_yaw(i); 
            end
        end
    end
end

figure();
ymin=-180;ymax = 180;
sampling_rate = 100;
ax(1) = subplot(2,1,1);
Time = (1:length(acc_filtered))/sampling_rate;
plot(Time, acc_filtered,'b');hold on;
plot(Time,9.8*ones(length(Time),1),'--r');hold on;
legend('Filtered Acc magnitude','Gravity');
xlabel('Second');ylabel('m/s^2');
plot(headingRecord(:,1)/sampling_rate,acc_filtered(headingRecord(:,1)),'ro');
for i=1:length(headingRecord(:,1))
    tmp= zeros(100,2);
    tmp(:,1) = headingRecord(i,1)/sampling_rate;
    tmp(:,2) = ymin:(ymax-ymin)/99:ymax;    
%     plot(tmp(:,1),tmp(:,2),'--','color',[0 0 0]);hold on;
end
ref_heading = median(headingRecord(:,2));
ax(2) = subplot(2,1,2);
plot(Time,AHRS_yaw,'b');hold on;
plot(Time,ref_heading*ones(length(Time),1),'--r');hold on;
legend('Heading', 'True Heading');
plot(headingRecord(:,1)/sampling_rate,headingRecord(:,2),'ro');
for i=1:length(headingRecord(:,1))
    tmp= zeros(100,2);
    tmp(:,1) = headingRecord(i,1)/sampling_rate;
    tmp(:,2) = ymin:(ymax-ymin)/99:ymax;    
%     plot(tmp(:,1),tmp(:,2),'--','color',[0 0 0]);hold on;
end
xlabel('Second');ylabel('Degree');
linkaxes(ax,'x');
hold off;

end

