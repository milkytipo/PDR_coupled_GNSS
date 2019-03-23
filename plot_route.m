function [ ] = plot_route( location_en, ref_enu )
switch nargin
    case 1
        ref_enu = 0;        
end 
figure();
plot_movie = false;
if plot_movie == true
    plotwidth=5;
    path = plot(location_en(1,1 ),location_en(1,2 ),'color','r','Marker','.','MarkerSize',plotwidth); grid on;hold on;
    axis([min(location_en(:,1)),max(location_en(:,1)),min(location_en(:,2)),max(location_en(:,2))]);
    set(path,'erasemode','none');
    for i =1:length(location_en(:,1))
        set(path ,'xdata',location_en(i,1 ),'ydata',location_en(i,2 ),'Marker','*','MarkerSize',plotwidth); 
        drawnow;pause(0.02)
    end	
end
if length(ref_enu(:,1)) > 1
    plot(location_en(:,1),location_en(:,2),'b');grid on;hold on;axis equal;
    plot(ref_enu(:,1),ref_enu(:,2),'r');hold on;
    legend('PDR route','Reference route from GPS');
    plot(location_en(1,1),location_en(1,2),'b*');hold on;
else
    %实验室地图和真值点
    load('labMapNew.mat')
    plot_map;
    hold on;
    plot (redCross(:,1), redCross(:,2), 'rx')
   
    %3D画图评测
%     load('F:\fused_pdr\data\floor3_points.mat');
%     load('F:\fused_pdr\data\floor1_points.mat');
%     plot3(floor3(:,1), floor3(:,2), floor3(:,3), 'r');hold on;
%     plot3(floor1(:,1), floor1(:,2), floor1(:,3), 'r')
    
    h_pdr = plot(location_en(:,1),location_en(:,2),'k');grid on;hold on;axis equal;
    h_start = plot(location_en(1,1),location_en(1,2),'bs');
    h_end = plot(location_en(end,1),location_en(end,2),'rs');
    legend([h_pdr h_start h_end],'PDR route','Start','End');
%     plot(location_en(1,1),location_en(1,2),'b*');hold on;
end

xlabel('x (m)');
ylabel('y (m)');
 hold off;
end

