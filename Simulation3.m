clear all
%close all
%%Initial value of states
y(1)=0;
y(2)=y(1);
yc(1)=y(1);
yc(2)=yc(1);
u(1)=0;
uc(1)=u(1);
%System Configuration
a1=-1.5001;
a2=0.4989;
a3=-0.5;
b1=2.8786;
b2=-0;
w_k=0.35;
dd(1)=0;
T=50;

%Control parameters
rho=0.2;
eta=0.4;
delta=10;
p=5;
ed_n=3;                                                               %ed_n=1,2,3 set up in sequence



for i=1:T+1
  d(i)=w_k*(0.6*sin(2*pi/10*i)+0.4*cos(2*pi/50*i));                      %Disturbance
  yr(i)=10;                                                           %yr(i)=10, 20, 30 set up in sequence
  if i>2
      dd(i)=d(i)-d(i-1);
  end
end
w_k2=max(abs(dd));                                                      % The upper bound of disturbances after differencing  
for i=1:T
    zero(i)=0;
        ss(i)=10*tan(eta/10+1.0142*atan(w_k2/10));
        
        e(i)=yr(i)-y(i);
        
        
       
        if i==2
            sys(i)=-a1*y(i)-a2*y(i-1)^2-a3*y(i)*y(i-1);
            if ed_n==1
                ed=(1-rho)*e(i)-eta*sign(e(i));                                                %A comparative approach 1
            elseif ed_n==2
                ed=(1-rho)*e(i)-eta*e(i)/(abs(e(i))+p);                                        %A comparative approach 2
            else
                ed=delta*tan((1-rho)*atan(e(i)/delta)-eta/(delta)*sign(e(i)));                 %Proposed error dynamics
            end
            u(i)=1/b1*(yr(i+1)+sys(i)-y(i)-ed);
            
            
        elseif i>2
            sys(i)=-a1*y(i)-a2*y(i-1)^2-a3*y(i)*y(i-1);
            dsys(i)=sys(i)-sys(i-1);
            if ed_n==1
                ed=(1-rho)*e(i)-eta*sign(e(i));
            elseif ed_n==2
                ed=(1-rho)*e(i)-eta*e(i)/(abs(e(i))+p);
            else
                ed=delta*tan((1-rho)*atan(e(i)/delta)-eta/(delta)*sign(e(i)));
            end
            u(i)=u(i-1)+1/b1*(yr(i+1)+dsys(i)-b2*(u(i-1)-u(i-2))-y(i)-ed);
        end
        
       
        if i>=2
            y(i+1)=a1*y(i)+a2*y(i-1)^2+a3*y(i)*y(i-1)+b1*u(i)+b2*u(i-1)-d(i);                            %System output
        end
end


figure(1)

t=1:T;
tt=t-1;
if ed_n==1
    l_r='k';
elseif ed_n==2
    l_r='b';
else
    l_r='r';
end
plot(tt,e(t),l_r,'linewidth',4.0);
hold on;
%plot(tt,zero(t),':k','linewidth',2.0);
hold on;
%plot(tt,ss(t),'--k',tt,-ss(t),'--k','linewidth',3.0);
leg = legend('y_d=10','y_d=20','y_d=30','y_d=10','y_d=20','y_d=30','y_d=10','y_d=20','y_d=30');leg.ItemTokenSize = [100,2];
%leg = legend('1','2','3','4');leg.ItemTokenSize = [100,2];
xlabel('\it{k}')
ylabel('\it{e_k}')
set(gca,'FontName','Times New Roman','FontSize',45)

figure(2)
if ed_n==1
    l_r='k';
elseif ed_n==2
    l_r='b';
else
    l_r='r';
end

plot(tt,u(t),l_r,'linewidth',4.0);
hold on;


leg = legend('y_d=10','y_d=20','y_d=30','y_d=10','y_d=20','y_d=30','y_d=10','y_d=20','y_d=30');leg.ItemTokenSize = [100,2];
xlabel('\it{k}')
ylabel('\it{u_k}')
set(gca,'FontName','Times New Roman','FontSize',45)

nss=ceil(log((0.1886*(0.9858*0.4/10+0.014)+0.9858*0.4/10-0.014)/((0.1886*pi/2+0.9858*0.4/10-0.014)))/log(1.0142*0.8))