function [top_matrix_regen, end_effector_regen] = end_effector_regeneration(r_p, theta_p)


%t1 = zeros(3,1);
t1 = [-r_p*sin(((2*pi/3) - theta_p)/2);0;-r_p*cos(((2*pi/3) - theta_p)/2)];

%for the second vertex
rotangle = (2*pi/3) - theta_p;
Ry_clock = [cos(rotangle) 0 -sin(rotangle);0 1 0;sin(rotangle) 0 cos(rotangle)];
t2 = Ry_clock*t1;

%for remaining vertices
rotangle = 2*pi/3;
Ry_clock = [cos(rotangle) 0 -sin(rotangle);0 1 0;sin(rotangle) 0 cos(rotangle)];
t3 = Ry_clock*t1;

%for point p4
t4 = Ry_clock*t2;
t5 = Ry_clock*t3;
t6 = Ry_clock*t4;

%changing frame
rotangle = ((2*pi/3) - theta_p)/2;
Ry_clock = [cos(rotangle) 0 -sin(rotangle);0 1 0;sin(rotangle) 0 cos(rotangle)];
top_matrix = [t2,t3,t4,t5,t6];

for i = 1:5
    top_matrix_regen(:,i) = Ry_clock*(top_matrix(:,i)-t1);
end
end_effector_regen = Ry_clock*([0;0;0]-t1);

