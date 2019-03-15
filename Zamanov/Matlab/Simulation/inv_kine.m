function [th, dth, ddth, J] = inv_kine(thi, dthi, ddthi)

[base_length, top_length, half_angle, rem_angle, Base_matrix, l1, L2, theta_p, theta_b, r_p] = parameters();
ex = thi(1);
ey = thi(2);
ez = thi(3);
roll = thi(4);
pitch = thi(5);
yaw = thi(6);

[Top_matrix] = end_effector(ex, ey, ez, roll, pitch, yaw, theta_p, r_p);

rangle_vectclock = [-pi/6, pi/6, -deg2rad(150), -pi/2, pi/2, deg2rad(150)];
VL = r_p;
for i = 1:6
     if i ==1
        k = 1;
    elseif i ==2
        k = 7;
    end
    Rx_clock = [1, 0, 0;0, cos(rangle_vectclock(i)), sin(rangle_vectclock(i));0, -sin(rangle_vectclock(i)), cos(rangle_vectclock(i))];
   
    T = Rx_clock*(Top_matrix(:,i)-Base_matrix(:,i));
    th(k+2) = asin(T(3)/L2);
    l2 = L2*cos(th(k+2));
    th(k+1) = acos((T(1)^2 + T(2)^2 - l1^2 - l2^2)/(2*l1*l2));
    beta = atan2(l2*sin(th(k+1)),(l1+l2*cos(th(k+1))));
    gamma = atan2(T(1),T(2));
    th(k) = gamma - beta;
    k = k+3;
end

%Spherical angles
theta_1d = pi/2 - th(1);
theta_2d = -th(2);
theta_3d = -th(3);
Rx_clock = [1, 0, 0;0, cos(rangle_vectclock(1)), sin(rangle_vectclock(1));0, -sin(rangle_vectclock(1)), cos(rangle_vectclock(1))];
Rz_clock_12 = [cos(theta_2d(1)+theta_1d(1)), sin(theta_2d(1)+theta_1d(1)), 0;-sin(theta_2d(1)+theta_1d(1)), cos(theta_2d(1) + theta_1d(1)), 0;0, 0, 1];
Ry_clock_3 = [cos(theta_3d(1)) 0 -sin(theta_3d(1));0 1 0;sin(theta_3d(1)) 0 cos(theta_3d(1))];

d_end_effector_wrt_vertex1rot = Ry_clock_3*Rz_clock_12*Rx_clock*([ex; ey; ez] - Top_matrix(:,1));
th(5) = -asin(d_end_effector_wrt_vertex1rot(3)/VL);
th(4) = atan2(d_end_effector_wrt_vertex1rot(1),d_end_effector_wrt_vertex1rot(2))-pi/2;

R_theta14 = [cos(-th(4)), -sin(-th(4)), 0;sin(-th(4)), cos(-th(4)), 0;0, 0, 1];
R_theta15 = [cos(th(5)) 0 sin(th(5));0 1 0;-sin(th(5)) 0 cos(th(5))];
vertex_2_wrt_vertex1 = Top_matrix(:,2)-Top_matrix(:,1); 
vertex_2_wrt_vertex1_rot = Ry_clock_3*Rz_clock_12*Rx_clock*vertex_2_wrt_vertex1;
vertex_2_interim = R_theta15'*R_theta14'*vertex_2_wrt_vertex1_rot;
[top_matrix_regen, end_effector_regen] = end_effector_regeneration(r_p, theta_p);
th(6) = -asin(vertex_2_interim(2)/top_matrix_regen(1,1));

J = analytical_jacobian(r_p, theta_p, l1, L2, th);
th0 = th;
for corr_i = 1:6
    if corr_i ==1
        corr_k = 1;
    elseif corr_i ==2
        corr_k = 7;
    end
    th(corr_k+2) = -th(corr_k+2);
    th(corr_k+1) = -th(corr_k+1);
    th(corr_k) = pi/2-th(corr_k);
    corr_k = corr_k+3;
end
th(5) = -pi/2 + th(5);
th(4) = -th(4);
G = G_RedySim(ex, ey, ez, roll, pitch, yaw, base_length, top_length, l1, L2, theta_b, theta_p, r_p, th0);
dth = zeros(size(th));
ddth = zeros(size(th));
end

