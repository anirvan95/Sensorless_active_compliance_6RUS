%% Inverse kinematics of a 6RUS Stewart manipulator
%Written by Anirvan Dutta and Durgesh Salunkhe

%%In this code the \theta(11,12) for all 6 links will be derived

%The Stewart manipulator has been decoupled to 5 legs of RU and a leg with
%RUS which will be connected to the end platform

%%Inverse kinematics for 3-link manipulator

function [theta_one, theta_two, theta_three, theta_14, theta_15, theta_16, M_matrix, K_matrix, d_end_effector_wrt_vertex1rot] = inverse_kinematics(Top_matrix, Base_matrix, l1, L2, ex, ey, ez, r_p, top_matrix_regen)

%rangle_vectclock = [-pi/6, pi/6 , deg2rad(210), -pi/2, pi/2, deg2rad(150)];
%rangle_vectclock = [deg2rad(150), -deg2rad(150), pi/6, pi/2, -pi/2, -pi/6];
rangle_vectclock = [-pi/6, pi/6, -deg2rad(150), -pi/2, pi/2, deg2rad(150)];
VL = r_p;
sign = -1;
for i = 1:6
    Rx_clock = [1, 0, 0;0, cos(rangle_vectclock(i)), sin(rangle_vectclock(i));0, -sin(rangle_vectclock(i)), cos(rangle_vectclock(i))];
   
    T = Rx_clock*(Top_matrix(:,i)-Base_matrix(:,i));
    theta_three(i) = asin(T(3)/L2);
    l2 = L2*cos(theta_three(i));
    theta_two(i) = acos((T(1)^2 + T(2)^2 - l1^2 - l2^2)/(2*l1*l2));
    beta = atan2(l2*sin(theta_two(i)),(l1+l2*cos(theta_two(i))));
    gamma = atan2(T(1),T(2));
    theta_one(i) = gamma - beta;

%     beta = atan2(l2*sin(theta_two(i)),(l1+l2*cos(theta_two(i))));
%     gamma = atan2(T(1),T(2));
%     theta_one(i) = gamma - beta;
%     r = sqrt(l1^2 + l2^2 + 2*l1*l2*cos(theta_two(i)));
%     phi = atan2((l1 + l2*cos(theta_two(i))),(l2*sin(theta_two(i))));
%     theta_one(i) = asin(T(1)/r)-phi;
    
    Rx_anti = [1, 0, 0;0, cos(rangle_vectclock(i)), -sin(rangle_vectclock(i));0, sin(rangle_vectclock(i)), cos(rangle_vectclock(i))];
    M_matrix(:,i) = (Rx_anti*[l1*sin(theta_one(i));l1*cos(theta_one(i));0])+Base_matrix(:,i);
    K_matrix(:,i) = (Rx_anti*[(l1*sin(theta_one(i))+l2*sin(theta_one(i)+theta_two(i)));(l1*cos(theta_one(i))+l2*cos(theta_one(i)+theta_two(i)));L2*sin(theta_three(i))]) + Base_matrix(:,i);
end

%Spherical angles
%theta14 about z axis
theta_1d = pi/2 - theta_one(1);
theta_2d = -theta_two(1);
theta_3d = -theta_three(1);
Rx_clock = [1, 0, 0;0, cos(rangle_vectclock(1)), sin(rangle_vectclock(1));0, -sin(rangle_vectclock(1)), cos(rangle_vectclock(1))];
Rz_clock_12 = [cos(theta_2d(1)+theta_1d(1)), sin(theta_2d(1)+theta_1d(1)), 0;-sin(theta_2d(1)+theta_1d(1)), cos(theta_2d(1) + theta_1d(1)), 0;0, 0, 1];
Ry_clock_3 = [cos(theta_3d(1)) 0 -sin(theta_3d(1));0 1 0;sin(theta_3d(1)) 0 cos(theta_3d(1))];

d_end_effector_wrt_vertex1rot = Ry_clock_3*Rz_clock_12*Rx_clock*([ex; ey; ez] - Top_matrix(:,1));
% theta_15 = -asin(d_end_effector_wrt_vertex1rot(3)/VL);
% theta_14 = pi/2 - asin(d_end_effector_wrt_vertex1rot(1)/(VL*cos(theta_15)));
theta_15 = -asin(d_end_effector_wrt_vertex1rot(3)/VL);
theta_14 = atan2(d_end_effector_wrt_vertex1rot(1),d_end_effector_wrt_vertex1rot(2))-pi/2;


R_theta14 = [cos(-theta_14), -sin(-theta_14), 0;sin(-theta_14), cos(-theta_14), 0;0, 0, 1];
R_theta15 = [cos(theta_15) 0 sin(theta_15);0 1 0;-sin(theta_15) 0 cos(theta_15)];
vertex_2_wrt_vertex1 = Top_matrix(:,2)-Top_matrix(:,1); 
vertex_2_wrt_vertex1_rot = Ry_clock_3*Rz_clock_12*Rx_clock*vertex_2_wrt_vertex1;
vertex_2_interim = R_theta15'*R_theta14'*vertex_2_wrt_vertex1_rot;

theta_16 = -asin(vertex_2_interim(2)/top_matrix_regen(1,1));
R_theta16 = [1 0 0; 0 cos(theta_16) -sin(theta_16); 0 sin(theta_16) cos(theta_16)];
end
