function [tu] = invdyn_tree_eff(th,dth,ddth,n,alp,a,b,bt,dx,dy,dz,m,g,Icxx,Icyy,Iczz,Icxy,Icyz,Iczx)
%This code was dveloped by Mr. Suril Shah
% Updated latest : 23 Nov 2010
% Number of for loops used is 2
% phi and tourque are input to this function
% This program takes care of gravitational acceleration.Input g is required
% in the inertial frame
g=-g;
% FORWARD RECURSION _FINDING TWIST AND TWIST RATE
%Initialization
% e=[0;0;1];
tt=zeros(3,n);
% tb=zeros(3,n);
dtt=zeros(3,n);
dtb=zeros(3,n);
tu=zeros(n,1);
twt=zeros(3,n);
twb=zeros(3,n);
tW=zeros(3,3,n);
cth=zeros(1,n);
sth=zeros(1,n);
cal=zeros(1,n);
sal=zeros(1,n);
Dxx=zeros(n,1);Dyy=zeros(n,1);Dzz=zeros(n,1);
Dxy=zeros(n,1);Dyz=zeros(n,1);Dzx=zeros(n,1);
Ixx=zeros(n,1);Iyy=zeros(n,1);Izz=zeros(n,1);
Ixy=zeros(n,1);Iyz=zeros(n,1);Izx=zeros(n,1);

% FOR LOOP STARTS
for i=1:n
    if bt(i)==0 %When parent of the link is ground link
        %w angular velocity
        tt(:,i)=[0;0;dth(i)];
        
        %v linear velocity
        %tb(:,i)=[0;0;0];
        
        %dw angular acceleration
        dtt(:,i)=[0;0;ddth(i)];
        om3=tt(3,i); dom3=dtt(3,i); om3s=om3*om3;
        tW(1,1,i)=-om3s;    tW(1,2,i)=-dom3;    tW(1,3,i)=0;
        tW(2,1,i)=dom3;     tW(2,2,i)=-om3s;    tW(2,3,i)=0;
        tW(3,1,i)=0;        tW(3,2,i)=0;        tW(3,3,i)=0;
        
        cal(i)=cos(alp(i));
        cth(i)=cos(th(i));
        sal(i)=sin(alp(i));
        sth(i)=sin(th(i));
        
        %dv linear acceleration
        %dtb(:,i)=Qi'*g;
        bx=g(1);by=g(2)*cal(i)+g(3)*sal(i);bz=-g(2)*sal(i)+g(3)*cal(i);
        dtb(1,i)=bx*cth(i)+by*sth(i);
        dtb(2,i)=-bx*sth(i)+by*cth(i);
        dtb(3,i)=bz;
    else %Calculation for the links other than those attached with ground
        cal(i)=cos(alp(i));
        cth(i)=cos(th(i));
        sal(i)=sin(alp(i));
        sth(i)=sin(th(i));
        %position vector from origin of link to origin of next link
        ai(1,1)=a(i);
        ai(2,1)=- b(i)*sal(i);
        ai(3,1)=b(i)*cal(i);
        
        %w angular velocity
        %Qi'*tt(:,bt(i))
        bx=tt(1,bt(i));by=tt(2,bt(i))*cal(i)+tt(3,bt(i))*sal(i);bz=-tt(2,bt(i))*sal(i)+tt(3,bt(i))*cal(i);
        ttb(1,1)=bx*cth(i)+by*sth(i);
        ttb(2,1)=-bx*sth(i)+by*cth(i);
        ttb(3,1)=bz;
        %tt(:,i)=Qi'*tt(:,bt(i))+e*dth(i);
        tt(1,i)=ttb(1);
        tt(2,i)=ttb(2);
        tt(3,i)=ttb(3)+dth(i);
        
        %v  linear velocity
        %ttbi=tt(:,bt(i));
        %ttbixai=[ttbi(2)*ai(3)-ai(2)*ttbi(3);-(ttbi(1)*ai(3)-ai(1)*ttbi(3));ttbi(1)*ai(2)-ai(1)*ttbi(2)];
        %tb(:,i)=Qi.'*(tb(:,bt(i))+ttbixai);
        
        %dw angular acceleration
        %Qi.'*dtt(:,bt(i))
        bx=dtt(1,bt(i));by=dtt(2,bt(i))*cal(i)+dtt(3,bt(i))*sal(i);bz=-dtt(2,bt(i))*sal(i)+dtt(3,bt(i))*cal(i);
        dttb(1,1)=bx*cth(i)+by*sth(i);
        dttb(2,1)=-bx*sth(i)+by*cth(i);
        dttb(3,1)=bz;
        %dtt(:,i)=Qi.'*dtt(:,bt(i))+e*ddth(i)+tt(:,i)xe*dth(i);
        dtt(1,i)=dttb(1)+tt(2,i)*dth(i);
        dtt(2,i)=dttb(2)-tt(1,i)*dth(i);
        dtt(3,i)=dttb(3)+ddth(i);
        
        om1=tt(1,i); dom1=dtt(1,i); om1s=om1*om1;
        om2=tt(2,i); dom2=dtt(2,i); om2s=om2*om2;
        om3=tt(3,i); dom3=dtt(3,i); om3s=om3*om3;
        om12=om1*om2;om23=om2*om3;om13=om1*om3;
        tW(1,1,i)=-om3s-om2s;   tW(1,2,i)=-dom3+om12;  tW(1,3,i)=dom2+om13;
        tW(2,1,i)=dom3+om12;    tW(2,2,i)=-om3s-om1s;  tW(2,3,i)=-dom1+om23;
        tW(3,1,i)=-dom2+om13;   tW(3,2,i)=dom1+om23;   tW(3,3,i)=-om2s-om1s;
        
        %dv linear acceleration
        
        %dtb(:,i)=Qi.'*(dtb(:,bt(i))+dttbixai+ttbixttbixai);
        %dtb(:,i)=Qi.'*(dtb(:,bt(i))+tW(:,:,bt(i))*ai);
        %dtb(:,i)=Qi'*dtbb
        %dtbb=dtb(:,bt(i))+tW(:,:,bt(i))*ai;
        dtbb(1)=dtb(1,bt(i))+tW(1,1,bt(i))*ai(1)+tW(1,2,bt(i))*ai(2)+tW(1,3,bt(i))*ai(3);
        dtbb(2)=dtb(2,bt(i))+tW(2,1,bt(i))*ai(1)+tW(2,2,bt(i))*ai(2)+tW(2,3,bt(i))*ai(3);
        dtbb(3)=dtb(3,bt(i))+tW(3,1,bt(i))*ai(1)+tW(3,2,bt(i))*ai(2)+tW(3,3,bt(i))*ai(3);
        
        
        bx=dtbb(1);by=dtbb(2)*cal(i)+dtbb(3)*sal(i);bz=-dtbb(2)*sal(i)+dtbb(3)*cal(i);
        dtb(1,i)=bx*cth(i)+by*sth(i);
        dtb(2,i)=-bx*sth(i)+by*cth(i);
        dtb(3,i)=bz;
    end
    
    % Transfer of inertia tencor form Center of mass to link origin
    dxxs=dx(i)*dx(i);dyys=dy(i)*dy(i);dzzs=dz(i)*dz(i);dxy=dx(i)*dy(i);dyz=dy(i)*dz(i);dzx=dz(i)*dx(i);
    Dxx(i)=-m(i)*(dzzs+dyys);   Dyy(i)=-m(i)*(dzzs+dxxs);   Dzz(i)=-m(i)*(dxxs+dyys);
    Dxy(i)=m(i)*dxy;            Dyz(i)=m(i)*dyz;            Dzx(i)=m(i)*dzx;
    Ixx(i)=Icxx(i)-Dxx(i);      Iyy(i)=Icyy(i)-Dyy(i);      Izz(i)=Iczz(i)-Dzz(i);
    Ixy(i)=Icxy(i)-Dxy(i);      Iyz(i)=Icyz(i)-Dyz(i);      Izx(i)=Iczx(i)-Dzx(i);
    %mass x d
    mdi(1,1)=m(i)*dx(i);
    mdi(2,1)=m(i)*dy(i);
    mdi(3,1)=m(i)*dz(i);
    
    %Wt Euler equations of motion
    dtbi(1,1)=dtb(1,i);
    dtbi(2,1)=dtb(2,i);
    dtbi(3,1)=dtb(3,i);
    mdixdtbi(1)=mdi(2)*dtbi(3)-dtbi(2)*mdi(3);
    mdixdtbi(2)=-(mdi(1)*dtbi(3)-dtbi(1)*mdi(3));
    mdixdtbi(3)=mdi(1)*dtbi(2)-dtbi(1)*mdi(2);
    
    
    dsum=0.5*(Ixx(i)+Iyy(i)+Izz(i));
    cI1=Ixx(i)-dsum;
    cI2=Iyy(i)-dsum;
    cI3=Izz(i)-dsum;
    ui(1,1)=Izx(i)*tW(2,1,i)-Ixy(i)*tW(3,1,i)+Iyz(i)*(tW(2,2,i)-tW(3,3,i))+cI3*tW(2,3,i)-cI2*tW(3,2,i);
    ui(2,1)=Ixy(i)*tW(3,2,i)-Iyz(i)*tW(1,2,i)+Izx(i)*(tW(3,3,i)-tW(1,1,i))+cI1*tW(3,1,i)-cI3*tW(1,3,i);
    ui(3,1)=Iyz(i)*tW(1,3,i)-Izx(i)*tW(2,3,i)+Ixy(i)*(tW(1,1,i)-tW(2,2,i))+cI2*tW(1,2,i)-cI1*tW(2,1,i);
    %Wit=Ii*dtt(:,i)+ttixItt+mdixdtbi=ui+mdixdtbi;
    Wit(1)=ui(1,1)+mdixdtbi(1);
    Wit(2)=ui(2,1)+mdixdtbi(2);
    Wit(3)=ui(3,1)+mdixdtbi(3);
    
    %twt(:,i)=Wit;
    twt(1,i)=Wit(1);
    twt(2,i)=Wit(2);
    twt(3,i)=Wit(3);
    
    %Wb Newton's equaitons of motion
    %Wib=m(i)*(dtb(:,i)+dttixdi+ttixttixdi)=m(i)*dtb(:,i)+tW(:,:,i)*mdi;
    Wib(1)=m(i)*dtb(1,i)+tW(1,1,i)*mdi(1)+tW(1,2,i)*mdi(2)+tW(1,3,i)*mdi(3);
    Wib(2)=m(i)*dtb(2,i)+tW(2,1,i)*mdi(1)+tW(2,2,i)*mdi(2)+tW(2,3,i)*mdi(3);
    Wib(3)=m(i)*dtb(3,i)+tW(3,1,i)*mdi(1)+tW(3,2,i)*mdi(2)+tW(3,3,i)*mdi(3);
    
    %twb(:,i)=Wib;
    twb(1,i)=Wib(1);
    twb(2,i)=Wib(2);
    twb(3,i)=Wib(3);
    
    
    
end

% BACKWARD RECURSION_FINDING JOINT TORQUE
for i=n:-1:1
    %Caluation of the generalized foces
    tu(i)=twt(3,i);
    if bt(i)~= 0% When parent of the link is not ground link
        ai(1,1)=a(i);
        ai(2,1)=- b(i)*sal(i);
        ai(3,1)=b(i)*cal(i);
        
        %twbi=Qi*twb(:,i);
        bx=twb(1,i)*cth(i)-twb(2,i)*sth(i);by=twb(1,i)*sth(i)+twb(2,i)*cth(i);bz=twb(3,i);
        twbi(1,1)=bx;
        twbi(2,1)=by*cal(i)-bz*sal(i);
        twbi(3,1)=by*sal(i)+bz*cal(i);
        %twti=Qi*twt(:,i);
        bx=twt(1,i)*cth(i)-twt(2,i)*sth(i);by=twt(1,i)*sth(i)+twt(2,i)*cth(i);bz=twt(3,i);
        twti(1,1)=bx;
        twti(2,1)=by*cal(i)-bz*sal(i);
        twti(3,1)=by*sal(i)+bz*cal(i);
        
        aixtwbi(1)=ai(2)*twbi(3)-twbi(2)*ai(3);
        aixtwbi(2)=-(ai(1)*twbi(3)-twbi(1)*ai(3));
        aixtwbi(3)=ai(1)*twbi(2)-twbi(1)*ai(2);
        
        %twt(:,bt(i))=twt(:,bt(i))+twti+aixtwbi;
        twt(1,bt(i))=twt(1,bt(i))+twti(1)+aixtwbi(1);
        twt(2,bt(i))=twt(2,bt(i))+twti(2)+aixtwbi(2);
        twt(3,bt(i))=twt(3,bt(i))+twti(3)+aixtwbi(3);
        %twb(:,bt(i))= twb(:,bt(i))+twbi;
        twb(1,bt(i))= twb(1,bt(i))+twbi(1);
        twb(2,bt(i))= twb(2,bt(i))+twbi(2);
        twb(3,bt(i))= twb(3,bt(i))+twbi(3);
    end
end