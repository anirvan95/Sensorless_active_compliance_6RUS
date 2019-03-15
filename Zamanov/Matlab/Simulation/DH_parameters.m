% clear all;
% close all;
% clc
% 
% theta11 = -1.2695;
% theta12 = 2.4361;
% theta13 = 0;
% theta14 = 1.9750 - pi/2;
% theta15 = -pi/2;
% theta16 = 0;
% 
% rem_angle = pi/2;
% l1 = 12;
% l2 = 24;
% VL = 10.6066;

	
function [Top_matrix_test, end_effector_test, Transformation] = DH_parameters(l1,l2,theta_one,theta_two,theta_three, theta_14, theta_15, theta_16, end_effector_regen, top_matrix_regen)

%rangle_vectclock = [deg2rad(150), -deg2rad(150), pi/6, pi/2, -pi/2, -pi/6];

alpha = [-pi/6 0 -pi/2 pi/2 -pi/2 -pi/2];
a = [0 l1 0 l2 0 0];
d = [0 0 0 0 0 0];
theta = [pi/2-theta_one(1) -theta_two(1) -theta_three(1) -theta_14 -pi/2+theta_15 theta_16];


Transformation = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];

for i = 1:6
	T_d = [1 0 0 0; 0 1 0 0; 0 0 1 d(i); 0 0 0 1];
	T_a = [1 0 0 a(i); 0 1 0 0; 0 0 1 0; 0 0 0 1];
	T_theta = [cos(theta(i)) -sin(theta(i)) 0 0; sin(theta(i)) cos(theta(i))  0 0; 0 0 1 0; 0 0 0 1];
	T_alpha = [1 0 0 0; 0 cos(alpha(i)) -sin(alpha(i)) 0; 0 sin(alpha(i)) cos(alpha(i)) 0; 0 0 0 1];	

	DH_matrix = T_alpha*T_a*T_theta*T_d;
	Transformation = Transformation*DH_matrix;
end

end_effector_test = Transformation*[end_effector_regen;1];
%end_effector_test = end_effector_test(1:3)+Base_matrix(:,1);
for i = 1:size(top_matrix_regen,2)
    Top_matrix_test(:,i) = Transformation*[top_matrix_regen(:,i);1];
    %Top_matrix_test(1:3,i) = Top_matrix_test(1:3,i) + Base_matrix(:,1);
end

