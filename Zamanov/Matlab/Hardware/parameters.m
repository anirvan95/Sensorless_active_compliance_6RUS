%% Parameters of a planar 6RUS parallel manipulator
%Written by Anirvan Dutta and Durgesh Salunkhe

%In this script the physical parameters of the manipulator will be defined

function [base_length, top_length, half_angle, rem_angle, Base_matrix, l1, L2, theta_p, r_p] = parameters()

% Parameters required for Ikin and Fkin


%Defining parameters
base_length = 0.142; %Length of bottom platform
top_length = 0.067; %Length of top platform
d_b = 0.23;
d_t = 0.25;
l1 = 0.075; %Length of link1
L2 = 0.23; %Length of link2

inter_value = ((d_b/2 + base_length/4)*2)/sqrt(3);
r_b = sqrt(inter_value^2 + (base_length/2)^2);
rem_angle = 2*asin(base_length/(2*r_b));
half_angle = (2*pi/3) - rem_angle;

inter_value2 = ((d_t/2 + top_length/4)*2)/sqrt(3);
r_p = sqrt(inter_value2^2 + (top_length/2)^2);
rem_angle_top = 2*asin(top_length/(2*r_p));
theta_p = (2*pi/3) - rem_angle_top;


%%Defining complete base platform
 b1 = zeros(3,1);
 b1(1) = 0;
 b1(2) = -base_length/(2*tan(rem_angle/2));
 b1(3) = -base_length/2;

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
 
