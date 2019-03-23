function [ update_points ] = StepCounter (acc_xyz, sampleFreq,doPlot)
% accMag = sum(acc_xyz'.^2).^0.5'*9.7937;
accMag=acc_xyz*9.7937;
% accMag=acc_xyz;
if doPlot == true
    figure;
    h_mag = plot(accMag);
    grid on;  %Íø¸ñ
    sf = num2str(sampleFreq);
    xlabel(strcat('sample points ', ' (', sf, ' Hz)'));
    ylabel('acceleration (m/s^2)');
end
dataSize = length(accMag);
threshold = 9.7937;
peak = threshold;
valley = threshold;
step_start = zeros(1,dataSize);
update_points = zeros(1,dataSize);

index = 1;
index_for_update = 1;
step_counter = 0;
step_candidate = 0;
tappingcounter = 0;
thresholdcounter = 0;

for i = 1:dataSize-2
    if accMag(i+1) > accMag(i) && accMag(i+1) < accMag(i+2) && accMag(i) < threshold && accMag(i+2) > threshold
        if index == 1
            step_start(index) = i+1;
            index = index + 1;
            step_candidate = step_candidate + 1;
        elseif (i+1-step_start(index-1)) >= (sampleFreq/3) && (i+1-step_start(index-1)) <= (2*sampleFreq) %%
            step_start(index) = i+1;        
            peak = max(accMag(step_start(index-1):step_start(index)));
            valley = min(accMag(step_start(index-1):step_start(index)));
            threshold = 0.5*(peak + valley); 

            index = index + 1;
            step_candidate = step_candidate + 1;

            p_t = peak - threshold;
            t_v = threshold - valley;
            if p_t > 0.2 && t_v > 0.7 %c++°æ±¾²âÊÔ
            %if p_t > 1.5 && t_v > 1.2%15 
                step_counter = step_counter + 1;
                
                update_points(index_for_update) = i+1;
                index_for_update = index_for_update + 1;
                if doPlot == true
                    hold on;
                    h_step = plot(i+1,accMag(i+1),'ro','LineWidth',3,'MarkerSize',8);
                end

                thresholdcounter = thresholdcounter + 1;
            else
                if doPlot == true
                    hold on;
                    plot(i+1,accMag(i+1),'go');
                end
                tappingcounter = tappingcounter + 1;
            end

        elseif (i+1-step_start(index-1)) > (2*sampleFreq)
            step_start(index) = i+1;
            index = index + 1;
            threshold = 9.7937;
        end
    elseif index>1 && (i-step_start(index-1))>(2*sampleFreq)
        threshold = 9.7937;
    end
end
% legend([h_mag h_step],'acc mag','step detected');
update_points = update_points(update_points>0);
disp(['The number of steps is:' num2str(step_counter)] );
% disp('The number of tapping is:');
% disp(tappingcounter);
% disp('The number of thresholdcounter is:');
% disp(thresholdcounter);

end
