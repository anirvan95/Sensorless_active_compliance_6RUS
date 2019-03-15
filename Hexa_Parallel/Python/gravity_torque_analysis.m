clc
clear all
close all
sign_correction = [-1,1,-1,1,-1,1];
time = xlsread('Current_Feedback_Feb28.xls','Time');
Current = xlsread('Current_Feedback_Feb28.xls','Current');
Load = xlsread('Current_Feedback_Feb28.xls','Load');
Load(:,1) = Load(:,1)*-1;
Load(:,3) = Load(:,3)*-1;
Load(:,5) = Load(:,5)*-1;
Current(:,1) = Current(:,1)*-1;
Current(:,3) = Current(:,3)*-1;
Current(:,5) = Current(:,5)*-1;
plot(time,Load)
%plot(time,Current)
for i=1:6
    val(i) = max(Current(:,i)) - min(Current(:,i));
end

