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
w_k=10;                                                        %w_k=0 or 10;
d_k=[w_k;0;0;0;0;0;0;0]; 

%Reference trajectory
xd=[1; 1-p; 1-p; 1-p; 1-p; 1-p; 1-p;1-p];
y_d=2000;                                                    % yd=2000,5000,8000 set up in sequence
x1d=xd*y_d;                                               

%Initial value of states
x(:,1)=[0;0;0;0;0;0;0;0;];
x1(:,1)=[0;0;0;0;0;0;0;0;];

%Control parameters
rho=0.2;
eta=15;
delta=3000;
num=50;


for i=1:num
    s(i)=c*(x1d-x(:,i));
    epsi(i)=atan(s(i)/delta);
    u(i)=(c*x1d-c*A*x(:,i)-delta*tan((1-rho)*epsi(i)-(eta/delta)*sign(s(i)))-c*d_k*sin(2*pi/20*(i-1)))/(c*b);  %Proposed control law
    x(:,i+1)=A*x(:,i)+b*u(i)+d_k*sin(2*pi/20*i);                                   %System output
end
for i=1:num
   ds(i)=c*d_k*(sin(2*pi/20*i)-sin(2*pi/20*(i-1)));                                                          
end
max_d=max(abs(ds));                          % The upper bound of disturbances after differencing  
for i=1:num
    s1(i)=c*(x1d-x1(:,i));
    epsi1(i)=s1(i);

    u1(i)=(c*x1d-c*A*x1(:,i)-((1-rho)*epsi1(i)-eta*sign(s1(i)))-c*d_k*sin(2*pi/20*(i-1)))/(c*b);       %A comparative approach
    x1(:,i+1)=A*x1(:,i)+b*u1(i)+d_k*sin(2*pi/20*i);
    
    
    yd(i)=x1d(1,:);
    if w_k==0
        ss(i)=eta;
    else
        ss(i)=eta+max_d;
    end
    sd(i)=0;
end
za=0.9989;  zb=1.0011;    %The upper and lower bounds of z

log((1-zb+zb*rho)*(za*eta/delta+atan(max_d/delta))+za*eta/delta-atan(max_d/delta)/((1-zb+zb*rho)*pi/2+za*eta/delta-atan(max_d/delta)))/log(zb*(1-rho))  % The upper bound of the convergence steps
                                             
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
leg = legend('y_d=2000','y_d=2000','y_d=5000','y_d=5000','y_d=8000','y_d=8000');leg.ItemTokenSize = [100,2];
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
leg = legend('y_d=2000','y_d=2000','y_d=5000','y_d=5000','y_d=8000','y_d=8000');leg.ItemTokenSize = [100,2];
xlabel('\it{k}')
ylabel('\it{x_{1,k}}')
set(gca,'FontName','Times New Roman','FontSize',45)
figure(3)
plot(tt,u(t),colorz,'linewidth',4.0);
hold on;
plot(tt,u1(t),colorz1,'linewidth',3.0);
hold on;
leg = legend('y_d=2000','y_d=2000','y_d=5000','y_d=5000','y_d=8000','y_d=8000');leg.ItemTokenSize = [100,2];
xlabel('\it{k}')
ylabel('\it{u_k}')
set(gca,'FontName','Times New Roman','FontSize',45)