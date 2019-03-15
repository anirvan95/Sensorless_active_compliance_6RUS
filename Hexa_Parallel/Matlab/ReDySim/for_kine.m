% ReDySim for_kine module. This module perform forward kinematics of the system under study
% Contibutors: Dr. Suril Shah and Prof S. K. Saha @IIT Delhi
function [so sc vc tt st]=for_kine(th, dth, n, alp, a, b, bt, dx, dy, dz)
[n dof type alp a b bt dx dy dz m g  Icxx Icyy Iczz Icxy Icyz Iczx aj al  ]=inputs() ;
%Initialization
e=[0;0;1];
z31=[0;0;0];
tt=zeros(3,n);
tb=zeros(3,n);
so=zeros(3,n);
sc=zeros(3,n);
st=zeros(3,n);
% st=zeros(3,n);
vc=zeros(3,n);
Qf=zeros(3,3,n);

% Enter location of fixed hinges
[base_length, top_length, half_angle, rem_angle, Base_matrix, l1, L2, theta_p, r_p] = parameters();
so(:,1)=Base_matrix(:,1);
so(:,7)=Base_matrix(:,2);
so(:,10)=Base_matrix(:,3);
so(:,13)=Base_matrix(:,4);
so(:,16)=Base_matrix(:,5);
so(:,19)=Base_matrix(:,6);

% FOR LOOP STARTS
for i=1:n
    if bt(i)==0 %When parent of the link is ground link
        
        Qi=[cos(th(i))              -sin(th(i))              0
            cos(alp(i))*sin(th(i))   cos(alp(i))*cos(th(i)) -sin(alp(i))
            sin(alp(i))*sin(th(i))   sin(alp(i))*cos(th(i))  cos(alp(i))];
        Qf(:,:,i)=Qi;
        %Positions
        di=[dx(i);dy(i);dz(i)];
        %         so(:,i)=[0;0;0];
        sc(:,i)=so(:,i)+Qf(:,:,i)*di;
        st(:,i)=sc(:,i)+Qf(:,:,i)*di;
        
        %w angular velocity
        tt(:,i)=e*dth(i);
        tti=tt(:,i);
        %v
        tb(:,i)=z31;
        ttixdi=[tti(2)*di(3)-di(2)*tti(3);-(tti(1)*di(3)-di(1)*tti(3));tti(1)*di(2)-di(1)*tti(2)];
        vc(:,i)=tb(:,i)+ttixdi;
    else %Calculation for the links other than those attached with ground
        Qi=[cos(th(i))              -sin(th(i))              0
            cos(alp(i))*sin(th(i))   cos(alp(i))*cos(th(i)) -sin(alp(i))
            sin(alp(i))*sin(th(i))   sin(alp(i))*cos(th(i))  cos(alp(i))];
        Qf(:,:,i)=Qf(:,:,bt(i))*Qi;
        
        %position vector from origin of link to origin of next link
        aim=[a(i)
            - b(i)*sin(alp(i))
            b(i)*cos(alp(i))];
        di=[dx(i);dy(i);dz(i)];
        
        %Positions
        so(:,i)=so(:,bt(i))+Qf(:,:,bt(i))*aim;
        sc(:,i)=so(:,i)+Qf(:,:,i)*di;
        st(:,i)=so(:,i)+Qf(:,:,i)*2*di;
        
        %w angular velocity
        tt(:,i)=Qi'*tt(:,bt(i))+e*dth(i);
        tti=tt(:,i);
        
        %v  linear velocity
        ttbi=tt(:,bt(i));
        ttbixaim=[ttbi(2)*aim(3)-aim(2)*ttbi(3);-(ttbi(1)*aim(3)-aim(1)*ttbi(3));ttbi(1)*aim(2)-aim(1)*ttbi(2)];
        tb(:,i)=Qi.'*(tb(:,bt(i))+ttbixaim);
        ttixdi=[tti(2)*di(3)-di(2)*tti(3);-(tti(1)*di(3)-di(1)*tti(3));tti(1)*di(2)-di(1)*tti(2)];
        vc(:,i)=tb(:,i)+ttixdi;
    end
end
end