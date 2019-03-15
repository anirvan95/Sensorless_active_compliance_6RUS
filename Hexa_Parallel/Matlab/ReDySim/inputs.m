% ReDySim inputs module. The model parameters are entered in this module
% Contibutors: Dr. Suril Shah and Prof S. K. Saha @IIT Delhi

function [n dof type alp a b bt dx dy dz m g  Icxx Icyy Iczz Icxy Icyz Iczx aj Tp steps]=inputs() 

%System: Fourbar Mechanism
% INPUTS
%Number of links 
n=21;

%Degree of fredom of the system
dof=6;

% Type of mechanism
type=1; % 1 for closed-loop and 0 for open-loop

%Actuated joints of open tree
aj=[1 0 0 0 0 0 1 0 0 1 0 0 1 0 0 1 0 0 1 0 0]; %enter 1 for actuated joints and 0 otherwise


%Time steps and span 
Tp=0.3; steps=0.1;

%DH PARAMETERs
[base_length, top_length, half_angle, rem_angle, Base_matrix, l1, L2, theta_p, r_p] = parameters();
VL = r_p;
rod_radius = 0.003;

rangle_vectclock = [pi/3, -pi/3, -pi/3, pi, pi, pi/3];
alp=[rangle_vectclock(1), 0, pi/2, -pi/2, -pi/2, -pi/2, rangle_vectclock(2), 0, pi/2, rangle_vectclock(3), 0, pi/2, rangle_vectclock(4), 0, pi/2, rangle_vectclock(5), 0, pi/2, rangle_vectclock(6), 0, pi/2];
a=[0 l1 0 L2 0 0 0 l1 0 0 l1 0 0 l1 0 0 l1 0 0 l1 0];
b=zeros(1,21);
%PARENT ARRAY
bt=[0 1 2 3 4 5 0 7 8 0 10 11 0 13 14 0 16 17 0 19 20]; 
length_array=[l1, 0, L2, 0, 0, VL, l1, 0, L2, l1, 0, L2, l1, 0, L2, l1, 0, L2, l1, 0, L2];
% d - VECTOR FORM ORIGIN TO CG 

%%%COG of end effector, about which frame??
dx=[l1/2, 0, L2/2, 0, 0, 0, l1/2, 0, L2/2, l1/2, 0, L2/2, l1/2, 0, L2/2, l1/2, 0, L2/2, l1/2, 0, L2/2];
dy=zeros(21);
dz=zeros(21);
dz(6) = VL/2;
% MASS AND MOMENT OF INERTIA AND GRAVITY
m=[0.3; 0; 0.6; 0; 0; 1; 0.3; 0; 0.6; 0.3; 0; 0.6; 0.3; 0; 0.6; 0.3; 0; 0.6; 0.3; 0; 0.6];
g=[-9.81; 0; 0];
% g=[0; 0; 0];

%Inertia Tensor of the kth link about Center-Of-Mass (COM) in ith frame
%which is rigid attach to the link
Icxx=zeros(n,1);Icyy=zeros(n,1);Iczz=zeros(n,1); % Initialization 
Icxy=zeros(n,1);Icyz=zeros(n,1);Iczx=zeros(n,1); % Initialization 
for count = 1:21
Icxx(count)=0.5*m(count)*rod_radius*rod_radius;   
Icyy(count)=(1/12)*m(count)*length_array(count)*length_array(count);  
Iczz(count)= Icyy(count);
end

% End_effector Inertia
Icxx(6)=(1*0.4*0.4)/2;   Icyy(6)= (1*0.4*0.4)/4;  Iczz(6)=(1*0.4*0.4)/4;


