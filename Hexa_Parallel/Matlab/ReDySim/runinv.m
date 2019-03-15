% ReDySim runinv module. This module perform inverse dynamics.
% Contibutors: Dr. Suril Shah and Prof S. K. Saha @IIT Delhi

function []=runinv()
disp('------------------------------------------------------------------');
disp('Recursive Inverse Dynamics');
disp('Contibutors: Dr. Suril Shah and Prof. S. K. Saha @IIT Delhi ');
disp('------------------------------------------------------------------');

% LOADING INPUT DATA FILE
[n dof type alp a b bt dx dy dz m g  Icxx Icyy Iczz Icxy Icyz Iczx aj Tp steps]=inputs();

fip1=fopen('tor.dat','w');
fip2=fopen('statevar.dat','w');
fip3=fopen('timevar.dat','w');

%Time countdown starts
len=round(Tp/steps+1);
tau_i=zeros(len,n);
lam=zeros(len,n-dof);
time=zeros(len,1);
Ja=zeros(dof,n);
ct=1;
for i=1:n
    if aj(i)==1
        Ja(ct,i)=1;
        ct=ct+1;
    end
end

count=0;
tic

for t=0:steps:Tp
    disp(t);
    count=count+1;
    if type ==1
        %Calculation of trejectories
        [thi dthi ddthi]= trajectory(t,dof,Tp);
        [th dth ddth J]=inv_kine(thi, dthi, ddthi);
        fprintf(fip2,'%e ',th, dth, ddth);fprintf(fip2,'\n');
        %Calculation of Driving forces and lamda
        [tu] = invdyn_tree_eff(th,dth,ddth,n,alp,a,b,bt,dx,dy,dz,m,g,Icxx,Icyy,Iczz,Icxy,Icyz,Iczx);
        %Method 1: Solve lamda and tau togather using I*dthh+C*dth=tau+J'*lamda
        %Define the (dof x n) matrix Ja such that tau=Ja'*tau_act, not that tau
        %has non zero element for the actuated joint only, others are zero
        F = [Ja' J'];
        tau_lam=pinv(F)*tu;
        %     end
        tau_i(count,:)=tau_lam;
        fprintf(fip1,'%e ',tau_i(count,:)); fprintf(fip1,'\n');
        %     %Method 2: Reduced equation to DOF of system by  J'(I*dthh+C*dth=tau)
        %     tau_i(count,1:dof)=JJ'*tu;
    elseif type==0
        %Calculation of trejectories
        [th dth ddth]= trajectory(t,dof,Tp);
        %Calculation of Driving forces and lamda
        [tu] = invdyn_tree_eff(th,dth,ddth,n,alp,a,b,bt,dx,dy,dz,m,g,Icxx,Icyy,Iczz,Icxy,Icyz,Iczx);
        tau_i(count,:)=tu;
    else
        error('Enter 1 for closed loop and 0 for open loop')
    end
    time(count,1)=t;
    fprintf(fip3,'%e ',time(count)); fprintf(fip3,'\n');
    i=i+1;
end
toc

% OPENING DATA FILE


% for i=1:len
%     fprintf(fip1,'%e ',tau_i(i,:)); fprintf(fip1,'\n');
%     %Calculation of trejectories
% 
%     if type ==1
%         [thi dthi ddthi]= trajectory(time(i),dof,Tp);
%         [th dth ddth]=inv_kine(thi, dthi, ddthi);
%     else
%         [th dth ddth]= trajectory(time(i),dof,Tp);
%     end
% 
%     fprintf(fip2,'%e ',th, dth, ddth);fprintf(fip2,'\n');
%     fprintf(fip3,'%e ',time(i)); fprintf(fip3,'\n');
% 
% end

disp('------------------------------------------------------------------');
disp('Recursive Inverse Dynamics');
disp('Contibutors: Dr. Suril Shah and Prof. S. K. Saha @IIT Delhi ');
disp('------------------------------------------------------------------');
end


