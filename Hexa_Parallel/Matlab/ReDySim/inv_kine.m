function [th, dth, ddth, J] = inv_kine(thi, dthi, ddthi)


[base_length, top_length, half_angle, rem_angle, Base_matrix, l1, L2, theta_p, r_p] = parameters();
ex = thi(1);
ey = thi(2);
ez = thi(3);
roll = thi(4);
pitch = thi(5);
yaw = thi(6);

[Top_matrix] = end_effector(ex, ey, ez, roll, pitch, yaw, theta_p, r_p);

rangle_vectclock = [pi/3, -pi/3, -pi/3, pi, pi, pi/3];
VL = r_p;


for i = 1:6
    if i ==1
        k = 1;
    elseif i ==2
        k = 7;
    end
    Rx_clock = [1, 0, 0;0, cos(rangle_vectclock(i)), sin(rangle_vectclock(i));0, -sin(rangle_vectclock(i)), cos(rangle_vectclock(i))];
    T = Rx_clock*(Top_matrix(:,i)-Base_matrix(:,i));
    th(k+2) = -asin(T(3)/L2);
    l2 = L2*cos(th(k+2));
    th(k+1) = acos((T(1)^2 + T(2)^2 - l1^2 - l2^2)/(2*l1*l2));
    r = sqrt(l1^2 + l2^2 + 2*l1*l2*cos(th(k+1)));
    phi = atan((l1 + l2*cos(th(k+1)))/(l2*sin(th(k+1))));
    th(k) = asin(T(1)/r)-phi;
    k = k+3;
end

%Spherical angles
%theta14 about z axis
end_eff = [ex; ey; ez];
Rx_clock = [1, 0, 0;0, cos(pi/3), sin(pi/3);0, -sin(pi/3), cos(pi/3)];
Rz_clock_12 = [cos(th(2) - th(1)), sin(th(2) - th(1)), 0;-sin(th(2) - th(1)), cos(th(2) - th(1)), 0;0, 0, 1];
Ry_clock_3 = [cos(th(3)) 0 -sin(th(3));0 1 0;sin(th(3)) 0 cos(th(3))];

d_end_effector_wrt_vertex1rot = Ry_clock_3*Rz_clock_12*Rx_clock*(end_eff - Top_matrix(:,1));
th(5) = -asin(d_end_effector_wrt_vertex1rot(3)/VL);
th(4) = pi/2 - asin(d_end_effector_wrt_vertex1rot(1)/(VL*cos(th(5))));
R_theta14 = [cos(th(4)), -sin(th(4)), 0;sin(th(4)), cos(th(4)), 0;0, 0, 1];
R_theta15 = [cos(th(5)) 0 sin(th(5));0 1 0;-sin(th(5)) 0 cos(th(5))];
vertex_2_wrt_vertex1 = Top_matrix(:,2)-Top_matrix(:,1);
vertex_2_wrt_vertex1_rot = Ry_clock_3*Rz_clock_12*Rx_clock*vertex_2_wrt_vertex1;
vertex_2_interim = R_theta15'*R_theta14'*vertex_2_wrt_vertex1_rot;
[top_matrix_regen, end_effector_regen] = end_effector_regeneration(r_p, theta_p);
th(6) = -asin(vertex_2_interim(2)/top_matrix_regen(1,1));

th0 = th;

for corr_i = 1:6
    if corr_i ==1
        corr_k = 1;
    elseif corr_i ==2
        corr_k = 7;
    end
    th(corr_k+2) = -th(corr_k+2);
    th(corr_k) = -th(corr_k);
    corr_k = corr_k+3;
end
th(5) = -pi/2 + th(5);

J = analytical_jacobian(r_p, theta_p, l1, L2, th);
vertex_2_wrt_vertex1_flat = top_matrix_regen(1,1);
G = G_RedySim(ex, ey, ez, roll, pitch, yaw, base_length, top_length, l1, L2, half_angle, rem_angle, th0, r_p, theta_p, vertex_2_wrt_vertex1_flat);
dth = G*dthi';
ddth = G*ddthi';

for corr_j = 1:6
    if corr_j ==1
        corr_k = 1;
    elseif corr_j ==2
        corr_k = 7;
    end
    dth(corr_k) = -dth(corr_k);
    ddth(corr_k) = -ddth(corr_k);
    dth(corr_k+2) = -dth(corr_k+2);
    ddth(corr_k+2) = -ddth(corr_k+2);
    corr_k = corr_k+3;
end
end

