clc
clear all
close all

%%Feedback plots

%Circular Trajectory - actual and theoretical torque values
load('Circular_tau.mat')
load('Circular_feedback.mat')
motor_no = 2;
plot(feedbackmodified(:,1),feedbackmodified(:,motor_no+1))
hold on;
plot(feedbackmodified(:,1),tauactuatedmodified(:,motor_no+1))
hold off;

%Vertical Trajectory - actual and theoretical torque values
load('Vertical_shm_feedback.mat')
load('Vertical_shm_tau.mat')
motor_no = 2;
plot(tauactuatedvertshm(:,1),feedbackvertical(:,motor_no+1))
hold on;
plot(tauactuatedvertshm(:,1),tauactuatedvertshm(:,motor_no+1))
hold off;

%Wrench Plot - X direction
load('WrenchX.mat')
plot(tauactuatedwrenchx1(:,2),'r')
hold on;
plot(tauactuatedwrenchx1(:,3),'g')
plot(tauactuatedwrenchx1(:,4),'b')
plot(tauactuatedwrenchx1(:,5),'c')
plot(tauactuatedwrenchx1(:,6),'m')
plot(tauactuatedwrenchx1(:,7),'k')
hold off;

%Wrench Plot - YZ direction
load('WrenchYZ.mat')
plot(tauactuatedwrenchyz(:,2),'r')
hold on;
plot(tauactuatedwrenchyz(:,3),'g')
plot(tauactuatedwrenchyz(:,4),'b')
plot(tauactuatedwrenchyz(:,5),'c')
plot(tauactuatedwrenchyz(:,6),'m')
plot(tauactuatedwrenchyz(:,7),'k')
hold off;