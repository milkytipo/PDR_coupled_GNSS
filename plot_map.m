function  plot_map()
global map;
% figure(101);
for i = 1:length(map.segment.sp(:,1))
    line([map.segment.sp(i,1),map.segment.ep(i,1)],[map.segment.sp(i,2),map.segment.ep(i,2)]);%axis equal;
    hold on;
end

% for i = 1:length(map.p_Arc)
%     theta = map.arc.sa(i):(pi/100):map.arc.ea(i);
%     x = map.arc.origin_x(i) + map.arc.r(i) * cos(theta);
%     y = map.arc.origin_y(i) + map.arc.r(i) * sin(theta);
%     plot(x,y,'r');
%     hold on;
% end

% hold off;
% for j = 1:length(map.p_Poly)
%     [m,n] = size(map.p_Poly{j,:});
%     map.poly.sp = [];
%     map.poly.ep = [];
%     for k = 1:m
%         map.poly.sp(end+1) = [map.p_Poly{j,:}(k)];
%         map.poly.ep(end+1) = [map.p_Poly{j,:}(m+k)];
%     end
%     plot(map.poly.sp,map.poly.ep);
%     hold on;
% end
end