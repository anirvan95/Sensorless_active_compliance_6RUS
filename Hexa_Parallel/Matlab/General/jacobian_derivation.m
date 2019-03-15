%%Code to derive Symbolic loop closure Jacobian for the ReDySim
clc
clear all
close all

%Design Paramters
syms r_p theta_p l1 l2
%Joint Space Co-ordinates
syms theta11 theta12 theta13 theta14 theta15 theta16 theta21 theta22 theta23 theta31 theta32 theta33 theta41 theta42 theta43 theta51 theta52 theta53 theta61 theta62 theta63


%Jacobian of first link
first_link_theta = [theta11; theta12; theta13; theta14; theta15; theta16];
[top_matrix_regen, end_effector_regen] = end_effector_regeneration(r_p, theta_p);
[Top_matrix_wrt_link1, End_effector_wrt_link1] = DH_parameters(l1,l2,theta11,theta12,theta13, theta14, theta15, theta16, end_effector_regen, top_matrix_regen);
joint_count = 1; 

for i = 1:3:15
    J(i:i+2,1:6) = -jacobian(Top_matrix_wrt_link1(1:3,joint_count),first_link_theta);
    joint_count = joint_count + 1;
end

 %Jacobian of rest links
theta_two = [theta22;theta32;theta42;theta52;theta62];
theta_one = [theta21;theta31;theta41;theta51;theta61];
theta_three = [theta23;theta33;theta43;theta53;theta63];

rangle_vectclock = [-pi/3, -pi/3, pi, pi, pi/3];
j = 7;
for i = 1:5
    pl2 = l2*cos(theta_three(i));
    Rx_anti = [1, 0, 0;0, cos(rangle_vectclock(i)), -sin(rangle_vectclock(i));0, sin(rangle_vectclock(i)), cos(rangle_vectclock(i))];
    Joint_cood = (Rx_anti*[(l1*cos(theta_one(i))+pl2*cos(theta_two(i)+theta_one(i)));(l1*sin(theta_one(i))+pl2*sin(theta_two(i)+theta_one(i)));l2*sin(theta_three(i))]);
    J(j-6:j-4,j:j+2) = jacobian(Joint_cood,[theta_one(i);theta_two(i);theta_three(i)]);
    j = j+3;
end

filestr = strcat('Jacobian.txt');
fid = fopen(filestr, 'wt');
for row = 1:15
    for column = 1:21
		fprintf(fid, 'J(%d,%d) = %s;\n', row, column, J(row,column));
	end
end
fclose(fid);
