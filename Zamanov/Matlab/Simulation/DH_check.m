clear all;
close all;
clc;

syms theta_14 theta_15 theta_16 al1 al2 al3
alpha = [0 0 pi/2 -al1 al2 al3];
a = [0 0 0 0 0 0];
d = [0 0 0 0 0 0 ];
theta = [0 0 0 theta_14 theta_15 theta_16];


Transformation = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];

for i = 4:6
	T_d = [1 0 0 0; 0 1 0 0; 0 0 1 d(i); 0 0 0 1];
	T_a = [1 0 0 a(i); 0 1 0 0; 0 0 1 0; 0 0 0 1];
	T_theta = [cos(theta(i)) -sin(theta(i)) 0 0; sin(theta(i)) cos(theta(i))  0 0; 0 0 1 0; 0 0 0 1];
	T_alpha = [1 0 0 0; 0 cos(alpha(i)) -sin(alpha(i)) 0; 0 sin(alpha(i)) cos(alpha(i)) 0; 0 0 0 1];	

	DH_matrix = T_alpha*T_a*T_theta*T_d;
	Transformation = Transformation*DH_matrix;
end

Top_matrix(2) - Top_matix(1)

