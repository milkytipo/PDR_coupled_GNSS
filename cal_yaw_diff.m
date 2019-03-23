function [ yaw_difference ] = cal_yaw_diff( q1,q2  )
q1_euler = quatern2euler(quaternConj(q1));
q2_euler = quatern2euler(quaternConj(q2));
q1_yaw = atan(tan(q1_euler(:,3)));
q2_yaw = atan(tan(q2_euler(:,3)));
d1 = norm((q1_euler(:,3)-mean(q1_euler(:,3))) - (q2_euler(:,3)-mean(q2_euler(:,3))));
d2 = norm((q1_yaw-mean(q1_yaw)) - (q2_yaw-mean(q2_yaw)));
yaw_difference = min(d1,d2)*180/pi;
end

