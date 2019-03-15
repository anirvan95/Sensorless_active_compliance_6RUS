function [M] = Workpiece_generation(roll, pitch, yaw, ex, ey, ez, breadth, length, height,unit)
% generate 3D cuboid of any orientation
% roll = 15;
% pitch = 30;
% yaw = 0;
% ex = 0;
% ey = 0;
% ez = 0;
% breadth = 10;
% length = 10;
% height = 10;
M = [];
R_roll = [1 0 0;0 cosd(roll) -sind(roll);0 sind(roll) cosd(roll)];
R_yaw = [cosd(yaw), -sind(yaw), 0;sind(yaw), cosd(yaw), 0;0, 0, 1];
R_pitch = [cosd(pitch) 0 sind(pitch);0 1 0;-sind(pitch) 0 cosd(pitch)];
axis([-50 50 -50 50 -50 50]);
Center_translate = [1 0 0 ex;0 1 0 ey;0  0 1 ez;0  0  0 1];
t1_final = [];
for i=0:unit:height
    for j=(-length/2):unit:(length/2)
        for k=(-breadth/2):unit:(breadth/2)
            t = [k;j;i];
            t1 = R_roll*R_pitch*R_yaw;
            t1_temp = Center_translate*[t;1];
            r1_temp = t1*t1_temp(1:3);
            r1_final = [r1_temp(1),r1_temp(2),r1_temp(3)];
            t1_final = t1*t1_temp(1:3);
%             scatter3(t1_final(1),t1_final(2),t1_final(3),'.r')
%             hold on;
%             pause(0)
            final_cood = [t1_final(1),t1_final(2),t1_final(3)];
            M = [M;final_cood];
        end
    end
end
%plot3(M(:,1),M(:,2),M(:,3),'.r');
%plot3(ex,ey,ez,'*b');
end