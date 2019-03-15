clear all;
close all;
clc;
rangle = 45;
	 
Rot_y = [cosd(rangle) 0 sind(rangle);0 1 0;-sind(rangle) 0 cosd(rangle)];
Point = Rot_y*[0;0;15]