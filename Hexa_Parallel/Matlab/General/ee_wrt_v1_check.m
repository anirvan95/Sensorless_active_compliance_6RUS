clear all;
close all;
clc
syms theta_16 a b c 
R_theta16 = [1 0 0; 0 cos(theta_16) -sin(theta_16); 0 sin(theta_16) cos(theta_16)];
Check = R_theta16*[a;b;c]
Ry_anti = [cos(pi/4) 0 sin(pi/4); 
	          0     1    0;
			  -sin(pi/4) 0 cos(pi/4)];
		  
Point_ee = Ry_anti*[0;0;15]