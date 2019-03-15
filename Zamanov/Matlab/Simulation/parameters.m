%% Parameters of a planar 6RSS parallel manipulator - Zamanov type
%Written by Anirvan Dutta and Durgesh Salunkhe

%In this script the physical parameters of the manipulator will be defined

function [base_length, top_length, half_angle, rem_angle, Base_matrix, l1, L2, theta_p, theta_b, r_p] = parameters()

% Parameters required for Ikin

%Defining parameters
base_length = 0.15; %Length of bottom platform
theta_b = deg2rad(45);
rem_angle = theta_b;
half_angle = (2*pi/3) - rem_angle; %Please refer Ikin_convention pic
r_b = base_length/(2*sin(rem_angle/2));


top_length = 0.12; %Length of top platform
theta_p = deg2rad(75);
r_p = top_length/(2*sin((2*pi/3-theta_p)/2));

l1 = 0.12; %Length of link1
L2 = 0.24; %Length of link2


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
 
