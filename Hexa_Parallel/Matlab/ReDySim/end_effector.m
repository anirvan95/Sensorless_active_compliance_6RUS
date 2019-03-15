%Defining end-effector for the current task space configuration

function [Top_matrix] = end_effector(ex, ey, ez, roll, pitch, yaw, theta_p, r_p)
    
    R_roll = [cos(roll), -sin(roll), 0;sin(roll), cos(roll), 0;0, 0, 1];
    R_yaw = [1, 0, 0;0, cos(yaw), -sin(yaw);0, sin(yaw), cos(yaw)];
    R_pitch = [cos(pitch) 0 sin(pitch);0 1 0;-sin(pitch) 0 cos(pitch)];
    
    % Shift end effector center point to top platform center point
    ex_new = [1 0 0 -ex;0 1 0 -ey;0 0 1 -ez;0 0 0 1]*[ex;ey;ez;1];
    
    %for 1st vertex 
    t1 = zeros(3,1);
    t1(1) = ex_new(1);
    t1(2) = ex_new(2)-r_p*cos(((2*pi/3) - theta_p)/2);
    t1(3) = ex_new(3)-r_p*sin(((2*pi/3) - theta_p)/2);
    
    %for the second vertex
    rotangle = (2*pi/3) - theta_p;
    Rx_clock = [1, 0, 0;0, cos(rotangle), sin(rotangle);0, -sin(rotangle), cos(rotangle)];
    t2 = Rx_clock*t1;
    
    %for remaining vertices
    rotangle = 2*pi/3;
    Rx_clock = [1, 0, 0;0, cos(rotangle), sin(rotangle);0, -sin(rotangle), cos(rotangle)];
    t3 = Rx_clock*t1;
    
    %for point p4
    t4 = Rx_clock*t2;
    t5 = Rx_clock*t3;
    t6 = Rx_clock*t4;
    
    t1 = R_roll*R_pitch*R_yaw*t1;
    t1_temp = [1 0 0 ex;0 1 0 ey;0 0 1 ez;0 0 0 1]*[t1;1];
    t1 = [t1_temp(1);t1_temp(2);t1_temp(3)];
    
    t2 = R_roll*R_pitch*R_yaw*t2;
    t2_temp = [1 0 0 ex;0 1 0 ey;0 0 1 ez;0 0 0 1]*[t2;1];
    t2 = [t2_temp(1);t2_temp(2);t2_temp(3)];
    
    t3 = R_roll*R_pitch*R_yaw*t3;
    t3_temp = [1 0 0 ex;0 1 0 ey;0 0 1 ez;0 0 0 1]*[t3;1];
    t3 = [t3_temp(1);t3_temp(2);t3_temp(3)];
    
    t4 = R_roll*R_pitch*R_yaw*t4;
    t4_temp = [1 0 0 ex;0 1 0 ey;0 0 1 ez;0 0 0 1]*[t4;1];
    t4 = [t4_temp(1);t4_temp(2);t4_temp(3)];
    
    t5 = R_roll*R_pitch*R_yaw*t5;
    t5_temp = [1 0 0 ex;0 1 0 ey;0 0 1 ez;0 0 0 1]*[t5;1];
    t5 = [t5_temp(1);t5_temp(2);t5_temp(3)];
    
    t6 = R_roll*R_pitch*R_yaw*t6;
    t6_temp = [1 0 0 ex;0 1 0 ey;0 0 1 ez;0 0 0 1]*[t6;1];
    t6 = [t6_temp(1);t6_temp(2);t6_temp(3)];
    
    Top_matrix = [t1, t2, t3, t4, t5, t6];
    


end