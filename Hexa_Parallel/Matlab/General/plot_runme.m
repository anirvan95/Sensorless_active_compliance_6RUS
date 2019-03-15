%%Plot of the mechanism

%Plotting the base
clc;
clear all;
close all;
[base_length, top_length, half_angle, rem_angle, Base_matrix, l1, L2, theta_p, r_p] = parameters();
[ex, ey, ez, roll, pitch, yaw, exdot, eydot, ezdot, rolldot, pitchdot, yawdot] = kintrajectory();


for i=1:length(ex)
    i
    [top_matrix_regen, end_effector_regen] = end_effector_regeneration(r_p, theta_p);
    vertex_2_wrt_vertex1_flat = top_matrix_regen(1,1);
    [Top_matrix] = end_effector(ex(i), ey(i), ez(i), roll(i), pitch(i), yaw(i), theta_p, r_p);
    [theta_one, theta_two, theta_three, theta_14, theta_15, theta_16, M_matrix, K_matrix, d_end_effector_wrt_vertex1rot] = inv_kin(Top_matrix, Base_matrix, l1, L2, ex(i), ey(i), ez(i), r_p, top_matrix_regen);
    [Top_matrix_test, end_effector_test, Transformation] = DH_parameters(l1,L2,-theta_one,theta_two,-theta_three, theta_14, -pi/2+theta_15, theta_16, end_effector_regen, top_matrix_regen);
    
    tt = Transformation*[0;0;0;1];
    x_test = Transformation*[0.05;0;0;1];
    y_test = Transformation*[0;0.05;0;1];
    z_test = Transformation*[0;0;0.05;1];
    % 	for j = 1:6
    % 		if j ==1
    % 			k = 1;
    % 		elseif j ==2
    % 			k = 7;
    % 		end
    % 		th0(k) = theta_one(j);
    % 		th0(k+1) = theta_two(j);
    % 		th0(k+2) = theta_three(j);
    % 		k = k+3;
    % 	end
    %
    % 	th0(4) = theta_14;
    % 	th0(5) = theta_15;
    % 	th0(6) = theta_16;
    %
    
    plot3([Base_matrix(1,:), Base_matrix(1,1)], [Base_matrix(2,:), Base_matrix(2,1)], [Base_matrix(3,:), Base_matrix(3,1)], 'Linewidth', 3);
    hold on;
    
    %%Plotting the end_effector
    plot3([Top_matrix(1,:), Top_matrix(1,1)], [Top_matrix(2,:), Top_matrix(2,1)], [Top_matrix(3,:), Top_matrix(3,1)],'k', 'Linewidth', 2);
    
    
    %%Plotting all links
    for loop_v = 1:6
        plot3([Base_matrix(1,loop_v), M_matrix(1,loop_v), K_matrix(1,loop_v)], [Base_matrix(2,loop_v), M_matrix(2,loop_v), K_matrix(2,loop_v)], [Base_matrix(3,loop_v), M_matrix(3,loop_v), K_matrix(3,loop_v)], 'r', 'Linewidth', 3);
    end
    
    grid on;
    
    % 	for k = 1:5
    % 		Top_matrix_test(1:3,k) = Top_matrix_test(1:3,k) + Base_matrix(:,1);
    % 		plot3(Top_matrix_test(1,k),Top_matrix_test(2,k),Top_matrix_test(3,k),'*k');
    % 	end
    % 	end_effector_test = end_effector_test(1:3)+Base_matrix(:,1);
    % 	plot3(end_effector_test(1),end_effector_test(2),end_effector_test(3),'*r');
    x_co = x_test(1:3) + Base_matrix(:,1);
    y_co = y_test(1:3) + Base_matrix(:,1);
    z_co = z_test(1:3) + Base_matrix(:,1);
    cent = tt(1:3) + Base_matrix(:,1);
    plot3(cent(1),cent(2),cent(3),'.c');
    plot3([cent(1),x_co(1)],[cent(2),x_co(2)],[cent(3),x_co(3)],'b','Linewidth', 2);
    plot3([cent(1),y_co(1)],[cent(2),y_co(2)],[cent(3),y_co(3)],'g','Linewidth', 2);
    plot3([cent(1),z_co(1)],[cent(2),z_co(2)],[cent(3),z_co(3)],'k','Linewidth', 2);
    hold off;
    %drawnow
    axis([-0.80 0.80 -0.80 0.80 -0.80 0.80]);
    % 	axis([-30 30 -50 50 -50 50]);
    view([90 0]);
    
    
%     % 	%%Checking determinant of jacobian
%     for j = 1:6
%         if j ==1
%             k = 1;
%         elseif j ==2
%             k = 7;
%         end
%         th0(k) = theta_one(j);
%         th(k) = -theta_one(j);
%         th0(k+1) = theta_two(j);
%         th(k+1) = theta_two(j);
%         th(k+2) = -theta_three(j);
%         th0(k+2) = theta_three(j);
%         k = k+3;
%     end
%     
%     th0(4) = theta_14;
%     th0(5) = theta_15;
%     th0(6) = theta_16;
%     th(4) = theta_14;
%     th(5) = -pi/2+theta_15;
%     th(6) = theta_16;
%     %
%     J = analytical_jacobian(r_p, theta_p, l1, L2, th);
%     G = G_RedySim(ex(i), ey(i), ez(i), roll(i), pitch(i), yaw(i), base_length, top_length, l1, L2, half_angle, rem_angle, th0, r_p, theta_p, vertex_2_wrt_vertex1_flat);
%     
%     task_dot = [exdot(i);eydot(i);ezdot(i);rolldot(i);pitchdot(i);yawdot(i)];
%     if i == 1
%         prevth = th;
%     else
%         thdot = G*task_dot;
%         for corr_j = 1:6
%             if corr_j ==1
%                 corr_k = 1;
%             elseif corr_j ==2
%                 corr_k = 7;
%             end
%             thdot(corr_k) = -thdot(corr_k);
%             thdot(corr_k+2) = -thdot(corr_k+2);
%             corr_k = corr_k+3;
%         end
%         %
%         %
%         %
%         thdot_an = (th-prevth);
%         prevth = th;
%         theta_dot = thdot';
%         error = thdot_an-thdot'
%         % 		jac_error = J*thdot;
%         % 		jac_error_an = J*thdot_an';
%         % 	end
%         % 	Gu = G([1,7,10,13,16,19],:);
%         %     J_det = [J(:,2:6),J(:,[8,9]),J(:,[11,12]),J(:,[14,15]),J(:,[17,18]),J(:,[20,21])];
%         % 	Gu_determinant = det(Gu)
%         % 	J_determinant = det(J_det)
%         
%    end
    pause(0.1)
end

