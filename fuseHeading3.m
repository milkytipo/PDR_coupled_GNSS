function fusedheading = fuseHeading3(yawHeading,PCAHeading)
yawHeading(:,2) = yawHeading(:,2)*pi/180;
yh = yawHeading(:,2);
ph = PCAHeading(:,2);
for i=2:length(yh)
    if yh(i)-yh(i-1) > 200*pi/180     , yh(i:end) = yh(i:end)-2*pi;
    elseif yh(i)-yh(i-1) < -200*pi/180, yh(i:end) = yh(i:end)+2*pi;
    end
    if ph(i)-ph(i-1) > 200*pi/180     , ph(i:end) = ph(i:end)-2*pi;
    elseif ph(i)-ph(i-1) < -200*pi/180, ph(i:end) = ph(i:end)+2*pi;
    end
end
std_len = 10;
std_yh = zeros(size(yh));
std_ph = zeros(size(ph));
for i=std_len+1:length(yh)
    std_yh(i) = std(yh(i-std_len+1:i));
    std_ph(i) = std(ph(i-std_len+1:i));
end
local_min = [];
for i=std_len/2+1:length(yh)-std_len/2
    if std_ph(i)<std_ph(i-1)&&std_ph(i)<std_ph(i+1)
        if std_ph(i) == min(std_ph(i-std_len/2+1:i+std_len/2))
            local_min(end+1) = i;
        end
    end
end

idx = intersect(find(std_ph<0.2),find(std_ph<std_yh));
idx = intersect(idx, local_min);
figure;
ax(1) = subplot(2,1,1);
plot(ax(1),std_yh,'r');hold on
plot(ax(1),std_ph,'b');legend('std of yaw heading','std of pca heading');
plot(idx,std_ph(idx),'ro');hold off;
ax(2) = subplot(2,1,2);
plot(ax(2),yh,'r');hold on;plot(ax(2),ph,'b');legend('yaw heading','pca heading');
plot(idx,ph(idx),'ro');hold off;
linkaxes(ax,'x');
%% fuse algorithm
fh = yh;
for i=1:length(idx)
    Idx = idx(i);
    bais = median(ph(Idx-std_len+1:Idx))- median(fh(Idx-std_len+1:Idx));
    if i == 1
        fh = fh+bais;
%     else
%         fh(Idx+1:end) = fh(Idx+1:end)+bais;
    end
end
fusedheading = yawHeading(:,1);
fusedheading(:,2) = mod(fh,2*pi);
for i=2:length(fh)
    if fh(i)-fh(i-1) > 200*pi/180     , fh(i:end) = fh(i:end)-2*pi;
    elseif fh(i)-fh(i-1) < -200*pi/180, fh(i:end) = fh(i:end)+2*pi;
    end
end

figure
ax(1) = subplot(3,1,1);
plot(ax(1),mod(yh,2*pi),'r');hold on;
legend('Yaw heading');
ax(2) = subplot(3,1,3);
plot(ax(2),mod(ph,2*pi),'b');
legend('PCA heading');
ax(3) = subplot(3,1,2);
plot(ax(3),mod(fh,2*pi),'m');linkaxes(ax,'x');
legend('fused heading');hold off;
end


        

