% ReDySim animate module. This module animates the system under study
% Contibutors: Dr. Suril Shah and Prof S. K. Saha @IIT Delhi
function [] = animate()
disp('------------------------------------------------------------------');
disp('Animating the simulation data');

load statevar.dat;
load timevar.dat;
Y=statevar;T=timevar;

[n dof type alp a b bt dx dy dz m g  Icxx Icyy Iczz Icxy Icyz Iczx aj al]=inputs();
len_sum=sum(al)*.5;
xmin=-len_sum;
xmax=len_sum;
ymin=-len_sum;
ymax=len_sum;
zmin=-len_sum;
zmax=len_sum;

figure('Name','Animation Window','NumberTitle','off');
for i=1:length(T)
    th=Y(i,1:n);
    dth=Y(i,n+1:2*n)';
    [so sc vc tt st]=for_kine(th, dth, n, alp, a, b, bt, dx, dy, dz)  ;
    for k = 1:21
        plot3([so(1,k),st(1,k)],[so(2,k),st(2,k)],[so(3,k),st(3,k)],'Linewidth', 3)
        hold on;
    end
    for k = 1:6
        if k ==1
            loop = 1;
        elseif k ==2
            loop = 7;
        end
        Base_dummy(1:3,k) = so(:,loop);
        loop = loop+3;
    end
    
    plot3([Base_dummy(1,:), Base_dummy(1,1)], [Base_dummy(2,:), Base_dummy(2,1)], [Base_dummy(3,:), Base_dummy(3,1)], 'Linewidth', 3);
    hold on;
    plot3(so(1,1:6), so(2,1:6), so(3,1:6), 'Linewidth', 3);
%     hold on;
%     plot3(so(1,7:9), so(2,7:9), so(3,7:9), 'Linewidth', 3);
%     hold on;
%     plot3(so(1,10:12), so(2,10:12), so(3,10:12), 'Linewidth', 3);
%     hold on;
    for dummy_var = 7:3:19
    plot3(so(1,dummy_var:dummy_var+2), so(2,dummy_var:dummy_var+2), so(3,dummy_var:dummy_var+2), 'Linewidth', 3);
    hold on;
    end
    axis([-0.50 0.50 -0.50 0.50 -0.50 0.50]);
    grid on;    
    %drawnow;
end