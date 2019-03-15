% ReDySim trajectory module. The desired indpendent joint trejectories are 
% enterd here
% Contibutors: Dr. Suril Shah and Prof S. K. Saha @IIT Delhi

function [thi dthi ddthi]= trajectory(tim, dof, Tp)
%Enter trejectories here

%Position
thi(1) = 0.25;%+0.01*tim;
thi(2) = 0*sin(tim);
thi(3) = 0*cos(tim);
thi(4) = 0.0; 
thi(5) = 0.0; 
thi(6) = 0.0; 

%Velocities
dthi(1) = 0.;
dthi(2) = 0*cos(tim);
dthi(3) = 0*sin(tim);
dthi(4) = 0; 
dthi(5) = 0; 
dthi(6) = 0; 

%Acceleration
ddthi(1) = 0;
ddthi(2) = -0*sin(tim);
ddthi(3) = -0*cos(tim);
ddthi(4) = 0; 
ddthi(5) = 0; 
ddthi(6) = 0;

end
