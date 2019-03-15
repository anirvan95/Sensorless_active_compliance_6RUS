%%High Pass Filter test
clc
clear all
close all

load('highpassfeedtest.mat')
load('highpasstautest.mat')

actualtau = feedbackfilewrenchx2(:,2:7);
hpf_tau = actualtau;
slope = zeros(size(actualtau));
%lpf_tau = sgolayfilt(actualtau,2,3);

h0 = 0.139207;
h1 = 0.4262528;
h2 = -0.4262528;
h3 = -0.1392073;

for i=4:size(actualtau,1)
    for j = 1:size(actualtau,2)
        hpf_tau(i,j) = h0*hpf_tau(i,j)+h1*hpf_tau(i-1,j)+h2*hpf_tau(i-2,j)+h3*hpf_tau(i-3,j);
        slope(i,j) = actualtau(i,j) - actualtau(i-1,j);
    end
end
plot(actualtau(:,1),'r')
hold on;
plot(hpf_tau(:,1),'b')
plot(abs(slope(:,2)),'k')
% %plot(time_vect,dxl_present_current_vect(:,1))
% current = 4.5*(dxl_present_current_vect - 2048)/1000;
% %hpf_current = zeros(length(current),1);
% std_current = zeros(length(current),1);
% h0 = -0.339207;
% h1 = -0.5262528;
% h2 = 0.5262528;
% h3 = 0.3392073;
% mean_current = sum(abs(current)')/6;
% hpf_current = mean_current;
% for i = 4:length(hpf_current)
%     %hpf_current(i) = (hpf_current(i)+hpf_current(i-1)+hpf_current(i-2))/3;
%     hpf_current(i) = h0*hpf_current(i) + h1*hpf_current(i-1) + h2*hpf_current(i-2) + h3*hpf_current(i-3);
%     
% end
% % bhi = fir1(3,0.4,'high',chebwin(35,30));
% % outhi = filter(bhi,5,hpf_current);
% plot(time_vect,abs(hpf_current))
% hold on;
% plot(time_vect,mean_current)