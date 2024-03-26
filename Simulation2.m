clear all

%System Configuration
p=0.94; a(1)=0; a(2)=0; a(3)=0.1;a(4)=0.2 ; a(5)=0;a(6)=0.5;a(7)=0.2;
A=[p,a(7),a(6),a(5),a(4),a(3),a(2),a(1);0,0,1,0,0,0,0,0;0,0,0,1,0,0,0,0;0,0,0,0,1,0,0,0;0,0,0,0,0,1,0,0;0,0,0,0,0,0,1,0;0,0,0,0,0,0,0,1;0,0,0,0,0,0,0,0;];
b=[0;0;0;0;0;0;0;1;];
c(1)=1;
for i=2:8
    c(i)=0;
   for j=1:i-1
       c(i)=c(i)+a(8-j)/(p^(i-j));
   end

end
w_k=10;
d_k=[w_k;0;0;0;0;0;0;0];

%Reference trajectory
xd=[1; 1-p; 1-p; 1-p; 1-p; 1-p; 1-p;1-p];
x1d=xd*5000;


%Initial value of states
x(:,1)=[0;0;0;0;0;0;0;0;];
x1(:,1)=[0;0;0;0;0;0;0;0;];
%Control parameters
rho=0.3;
eta=15;
delta=1000;                             %delta=1000, 3000, 8000 set up in sequence
num=50;

for i=1:num
    s(i)=c*(x1d-x(:,i));
    epsi(i)=atan(s(i)/delta);
    u(i)=(c*x1d-c*A*x(:,i)-delta*tan((1-rho)*epsi(i)-(eta/delta)*sign(s(i)))-c*d_k*sin(2*pi/20*(i-1)))/(c*b); %Proposed switching control law
    x(:,i+1)=A*x(:,i)+b*u(i)+d_k*sin(2*pi/20*i);                                  %System output
end
for i=1:num
   ds(i)=c*d_k*(sin(2*pi/20*i)-sin(2*pi/20*(i-1)));
end
max_d=max(abs(ds));                                            % The upper bound of disturbances after differencing  
for i=1:num
    s1(i)=c*(x1d-x1(:,i));
    epsi1(i)=atan(s1(i)/delta);
    
    u1(i)=(c*x1d-c*A*x1(:,i)-delta*tan((1-rho)*epsi1(i))-c*d_k*sin(2*pi/20*(i-1)))/(c*b); %Proposed non-switching control law

    x1(:,i+1)=A*x1(:,i)+b*u1(i)+d_k*sin(2*pi/20*i);
    
    
    yd(i)=x1d(1,:);
    if w_k==0
        ss(i)=eta;
    elseif eta==0
        ss(i)=max_d/rho;
    else
        ss(i)=eta+max_d;
    end
    
    sd(i)=0;
end
colorz='r';
colorz1='b';
y=x(1,:);
y1=x1(1,:);

t=1:num;
tt=t-1;
figure(1)
plot(tt,s(t),colorz,'linewidth',4.0);
hold on;
plot(tt,s1(t),colorz1,'linewidth',3.0);
hold on;
%plot(tt,ss(t),'--k',tt,-ss(t),'--k','linewidth',3.0);
hold on;
%plot(tt,sd(t),':k','linewidth',2.0);
hold on;
leg = legend('\delta=1000','\delta=1000','\delta=3000','\delta=3000','\delta=8000','\delta=8000');leg.ItemTokenSize = [100,2];
xlabel('\it{k}')
ylabel('\it{s_k}')
set(gca,'FontName','Times New Roman','FontSize',45)



figure(2)
%plot(tt,yd(t),'--k');
hold on;
plot(tt,y(t),colorz,'linewidth',4.0);
hold on;
plot(tt,y1(t),colorz1,'linewidth',3.0);
hold on;
leg = legend('\delta=1000','\delta=1000','\delta=3000','\delta=3000','\delta=8000','\delta=8000');leg.ItemTokenSize = [100,2];
xlabel('\it{k}')
ylabel('\it{x_{1,k}}')
set(gca,'FontName','Times New Roman','FontSize',45)
figure(3)
plot(tt,u(t),colorz,'linewidth',4.0);
hold on;
plot(tt,u1(t),colorz1,'linewidth',3.0);
hold on;
leg = legend('\delta=1000','\delta=1000','\delta=3000','\delta=3000','\delta=8000','\delta=8000');leg.ItemTokenSize = [100,2];
xlabel('\it{k}')
ylabel('\it{u_k}')
set(gca,'FontName','Times New Roman','FontSize',45)