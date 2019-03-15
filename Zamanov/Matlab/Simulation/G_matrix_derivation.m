clear all;
close all;
clc

syms ex ey ez roll pitch yaw base_length top_length l1 L2 theta_b theta_p r_p
syms theta11 theta12 theta13 theta14 theta15 theta16 theta21 theta22 theta23 theta31 theta32 theta33
syms theta41 theta42 theta43 theta51 theta52 theta53 theta61 theta62 theta63

[top_matrix_regen, end_effector_regen] = end_effector_regeneration(r_p, theta_p);

rem_angle = theta_b;
half_angle = (2*pi/3) - rem_angle;

%%Defining complete base platform
b1 = [0;-base_length/(2*tan(rem_angle/2));-base_length/2];

%Defining the second vertex
rotangle = rem_angle;
Rx_clock = [1, 0, 0;0, cos(rotangle), sin(rotangle);0, -sin(rotangle), cos(rotangle)];
b2 =  Rx_clock*b1;

%Defining the rest vertices
rotangle = rem_angle+half_angle;
Rx_clock = [1, 0, 0;0, cos(rotangle), sin(rotangle);0, -sin(rotangle), cos(rotangle)];
b3 = Rx_clock*b1;
b4 = Rx_clock*b2;
b5 = Rx_clock*b3;
b6 = Rx_clock*b4;

Base_matrix = [b1,b2,b3,b4,b5,b6];

R_roll = [cos(roll), -sin(roll), 0;sin(roll), cos(roll), 0;0, 0, 1];
R_yaw = [1, 0, 0;0, cos(yaw), -sin(yaw);0, sin(yaw), cos(yaw)];
R_pitch = [cos(pitch) 0 sin(pitch);0 1 0;-sin(pitch) 0 cos(pitch)];

% Shift end effector center point to top platform center point
ex_new = [1 0 0 -ex;0 1 0 -ey;0 0 1 -ez;0 0 0 1]*[ex;ey;ez;1];

%for 1st vertex
t1 = [ex_new(1); ex_new(2)-r_p*cos(((2*pi/3) - theta_p)/2); ex_new(3)-r_p*sin(((2*pi/3) - theta_p)/2)];


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

rangle_vectclock = [-pi/6, pi/6, -deg2rad(150), -pi/2, pi/2, deg2rad(150)];
VL = r_p;

for i = 1:6
    if i ==1
        k = 1;
    elseif i ==2
        k = 7;
    end
    Rx_clock = [1, 0, 0;0, cos(rangle_vectclock(i)), sin(rangle_vectclock(i));0, -sin(rangle_vectclock(i)), cos(rangle_vectclock(i))];
    T_vect(:,i) = Rx_clock*(Top_matrix(:,i)-Base_matrix(:,i));
    T = Rx_clock*(Top_matrix(:,i)-Base_matrix(:,i));
    q(k+2) = asin(T(3)/L2);
    l2 = L2*cos(q(k+2));
    q(k+1) = acos((T(1)^2 + T(2)^2 - l1^2 - l2^2)/(2*l1*l2));
    beta = atan(l2*sin(q(k+1))/(l1+l2*cos(q(k+1))));
    gamma = atan(T(1)/T(2));
    q(k) = gamma - beta;
    
    k = k+3;
end

%Spherical angles
%theta14 about z axis
theta_1d = pi/2 - q(1);    %% This value will be a +ve clockwise rotation value
theta_2d = -q(2);          %% ??
theta_3d = -q(3);
Rx_clock = [1, 0, 0;0, cos(rangle_vectclock(1)), sin(rangle_vectclock(1));0, -sin(rangle_vectclock(1)), cos(rangle_vectclock(1))];
Rz_clock_12 = [cos(theta_2d(1)+theta_1d(1)), sin(theta_2d(1)+theta_1d(1)), 0;-sin(theta_2d(1)+theta_1d(1)), cos(theta_2d(1) + theta_1d(1)), 0;0, 0, 1];
Ry_clock_3 = [cos(theta_3d(1)) 0 -sin(theta_3d(1));0 1 0;sin(theta_3d(1)) 0 cos(theta_3d(1))];


d_end_effector_wrt_vertex1rot = Ry_clock_3*Rz_clock_12*Rx_clock*([ex; ey; ez] - Top_matrix(:,1));
q(5) = -asin(d_end_effector_wrt_vertex1rot(3)/VL);
q(4) = atan(d_end_effector_wrt_vertex1rot(1)/d_end_effector_wrt_vertex1rot(2))-pi/2;


R_theta14 = [cos(-q(4)), -sin(-q(4)), 0;sin(-q(4)), cos(-q(4)), 0;0, 0, 1];
R_theta15 = [cos(q(5)) 0 sin(q(5));0 1 0;-sin(q(5)) 0 cos(q(5))];
vertex_2_wrt_vertex1 = Top_matrix(:,2)-Top_matrix(:,1);
vertex_2_wrt_vertex1_rot = Ry_clock_3*Rz_clock_12*Rx_clock*vertex_2_wrt_vertex1;
vertex_2_interim = R_theta15'*R_theta14'*vertex_2_wrt_vertex1_rot;

q(6) = -asin(vertex_2_interim(2)/top_matrix_regen(1,1));

%G_matrix derivation

task_space = [ex; ey; ez; roll; pitch; yaw];
theta_one = [theta11; theta21; theta31; theta41; theta51; theta61];
theta_two = [theta12; theta22; theta32; theta42; theta52; theta62];
theta_three = [theta13; theta23; theta33; theta43; theta53; theta63];

for i = 1:6
    
    if i ==1
        k = 1;
    elseif i ==2
        k = 7;
    end
    
    for j = 1:6
        
%         G((k+2),j) = diff(T_vect(3,i),task_space(j))/(L2*cos(theta_three(i)));  %% Edit 1
		G((k+2),j) = -diff(T_vect(3,i),task_space(j))/(L2*cos(theta_three(i)));
        l2 = L2*cos(theta_three(i));
		l2dot = -L2*sin(theta_three(i))*G((k+2),j);

		B = -2*l1*L2*sin(theta_three(i))*cos(theta_two(i));
		C = -L2*L2*sin(2*theta_three(i));
		D = -2*l1*L2*sin(theta_two(i))*cos(theta_three(i));
		E = 2*T_vect(1,i)*diff(T_vect(1,i),task_space(j));
		F = 2*T_vect(2,i)*diff(T_vect(2,i),task_space(j));
		
		G(k+1,j) = -(E + F - (B+C)*G((k+2),j))/D;
        
%         tan(alpha) = T_vect(1,i)/T_vect(2,i);
%         tan(beta) = l2*sin(theta_two(i))/(l1+l2*cos(theta_two(i)));
		beta = atan2(l2*sin(theta_two(i)),(l1+l2*cos(theta_two(i))));
		gamma = atan2(T(1),T(2));
		
		H = l2dot*sin(theta_two(i)) + l2*cos(theta_two(i))*G(k+1,j);
		I = (l2*sin(theta_two(i))/(l1+l2*cos(theta_two(i))))*(l2dot*cos(theta_two(i)) - l2*sin(theta_two(i))*G(k+1,j));
		%%sec^2 = 1 + tan^2
		L = 1 + ((l2*sin(theta_two(i)))/((l1+l2*cos(theta_two(i)))))^2; %% sec(beta)^2
		beta_dot = (H - I)/(L*(l1 + l2*cos(theta_two(i))));
		
		M = 1 + ((T(1))/(T(2)))^2;  %% sec(gamma)^2
		gamma_dot = (diff(T_vect(1,i),task_space(j))*T_vect(2,i) - diff(T_vect(2,i),task_space(j))*T_vect(1,i))/(M*T_vect(2,i)*T_vect(2,i));
        
		G(k,j) = beta_dot - gamma_dot;  
        
    end
    k = k+3;
end

for j = 1:6
    a = d_end_effector_wrt_vertex1rot(1);
    b = d_end_effector_wrt_vertex1rot(2);
    c = d_end_effector_wrt_vertex1rot(3);
    
    G(5,j) = -diff(c,task_space(j))/(VL*cos(theta15));
    G(4,j) = -sin(theta14)^2*((diff(a,task_space(j))*b-diff(b,task_space(j))*a)/(b*b));
    G(6,j) = -diff(vertex_2_interim(2),task_space(j))/(top_matrix_regen(1,1)*cos(theta16));
end
%simplification 
for row = 4
	filestr = strcat('G_files/G_RedySim.txt', num2str(row), '.txt');
	fid = fopen(filestr, 'wt');
	for column = 1:6
		fprintf(fid, 'G(%d,%d) = %s;\n', row, column, G(row,column));
	end
	fclose(fid);
end