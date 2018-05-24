%% Discrete Ballbot LQR controls
% by Adam Woo

clear all; close all; clc;

m = 7.2;        % mass of chassis (kg)
M = .85;        % mass of ball (kg)
b = .1;
l = .2;         % length to center mass of chassis (m)
g = -9.81;      % gravity (m/s)
d = 2;          % damping factor
I = M*l*b;
start = 0;      % x-axis initial location (m)
dest = 0;       % x-axis destination (m)
pushx = .1%rand(1)*(pi/4)-rand(1)*(pi/4);   % initial angle from verticle (rads)
pushy = .1%rand(1)*(pi/4)-rand(1)*(pi/4);   % initial angle from verticle (rads)
r=0;
noise=0;    % add noise if 1

t=10;                 % Simulation Duration (seconds)
Ts=0.01;          % Sampling Interval (seconds)
tspan=(0:Ts:t)';
tlen=length(tspan);


p = I*(M+m)+M*m*l^2; %denominator for the A and B matrices

A = [0      1              0           0;
     0 -(I+m*l^2)*b/p  (m^2*-g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p    2*m*-g*l*(M+m)/p  0]

B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p]

% A = [0        0           1           0;
%      0        0         -m*g/M        0;
%      0        0           0           1;
%      0        0     -(m+M)*g/(M*l)    0]
% 
%  B = [     0;
%          1/M;
%            0;
%      1/(M*l)]

% 
% A = [0          1             0             0;
%     0         -d/M         -m*g/M           0;
%     0           0             0             1;
%     0        -d/(M*l)   -(m+M)*g/(M*l)      0]
% 
% B = [  0;
%       1/M;
%        0;
%      1/(M*l)]
% 
% 
% C = [0 0 1 0;
%      0 0 0 1];
% D = [0;
%      0];
% 
% states = {'x' 'x_dot' 'phi' 'phi_dot'};
% inputs = {'u'};
% outputs = {'x'; 'phi'};
% 
% sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
% 
% sys_d = c2d(sys_ss,Ts,'zoh')


C = [0   0   0   0;
    0   0   0   0;
    0   0   1   0;
     0   0   0   1];

D = [0;
    0;
    0;
     0];
 
Qs = [1 0 0 0;
      0 10 0 0;
      0 0 1000 0;
      0 0 0 100];

% Ql = [0 0 0 0;
%     0 10 0 0;
%     0 0 10 0;
%     0 0 0 10];

R = .001;

[num,den] = ss2tf(A,B,C,D);

K=zeros(1,size(A,1));

Ks = lqrd(A,B,Qs,R,Ts)

% Kl = lqrd(A,B,Ql,R,Ts)

% L = [0.1819;
%     0.0161;
%     0.0000;
%    -0.0002]

%% Discrete simulation

xx=zeros(tlen,size(A,1));
xxdot=zeros(tlen,size(A,1));
xx(1,:) = [start; 0; pi; 0];
xy=zeros(tlen,size(A,1));
xydot=zeros(tlen,size(A,1));
xy(1,:) = [start; 0; pi; 0];

% x_hat=zeros(tlen,size(A,1));
% x_hat(1,:) = [start; 0; pi+rand(1); 0];

yx=zeros(tlen,size(C,1));
yy=zeros(tlen,size(C,1));

% y(1,2)=pi+angle;
% y_hat=zeros(tlen,size(C,1));

motor=zeros(tlen,3);


xx(1,3)=xx(1,3)+(pushx);
xy(1,3)=xy(1,3)+(pushy);
        
for i = 2:tlen
    
    if noise == 1
        % noise input
        xx(i-1,3)=xx(i-1,3)+(rand(1)*(pi/128))-(rand(1)*(pi/128));
        xy(i-1,3)=xy(i-1,3)+(rand(1)*(pi/128))-(rand(1)*(pi/128));
        xx(i-1,4)=xx(i-1,4)+(rand(1)*(pi/64))-(rand(1)*(pi/64));
        xy(i-1,4)=xy(i-1,4)+(rand(1)*(pi/64))-(rand(1)*(pi/64));
    end
    
    % error calculation
    errorx = xx(i-1,:)'-[xx(i-1,1); 0; pi; 0];
    errory = xy(i-1,:)'-[dest; 0; pi; 0];
    
    % state space control
    if (abs((xx(i-1,3)-pi)*(180/pi)) > 5)
%         K=K-(K-Kl)/2;
    else
        K=K-(K-Ks)/2;
    end
    xxdot(i,:)=(((A-B*Ks)*(errorx))*Ts);
    xx(i,:)=(((A-B*Ks)*(errorx))*Ts)+xx(i-1,:)';
    yx(i,:)=xx(i,:)*C';
    
    if (abs((xy(i-1,3)-pi)*(180/pi)) > 5)
%         K=K-(K-Kl)/2;
    else
        K=K-(K-Ks)/2;
    end
    xydot(i,:)=(((A-B*Ks)*(errory))*Ts);
    xy(i,:)=(((A-B*Ks)*(errory))*Ts)+xy(i-1,:)';    
    yy(i,:)=xy(i,:)*C';

    %TODO: Full State Estimator  
%     u=(-K*x_hat(i-1,:)')+r;
%     x_hat(i,:)=(((A*x_hat(i-1,:)')+(B*u))*Ts)+L*(y(i-1)-y_hat(i-1));
%     y_hat(i)=x(i,:)*C';
% 
%     x(i,:)=(((A*x(i-1,:)')+(B*u))*Ts);
%     y(i)=x(i,:)*C';

ax=xxdot(i,2);
ay=xydot(i,2);
az=0;

motor(i,1) = (((1/3)*ax)+((-1/sqrt(3))*ay)+((1/3)*az));
motor(i,2) = (((1/3)*ax)+((1/sqrt(3))*ay)+((1/3)*az));
motor(i,3) = (((-2/3)*ax)+((1/3)*az));

motor(i,1) = motor(i-1,1);
motor(i,2) = motor(i-1,2);
motor(i,3) = motor(i-1,3);


end



%% Draw Simulation
close all;
for k=1:1/(10*Ts):length(tspan)
    drawballbot_2D(xx(k,:),xy(k,:),m,M,l,t,tspan(1:k),motor(1:k,:));
end

% figure;
% subplot(3,1,1);
% plot(tspan,m1);
% title('Motor Velocities with State-Feedback Control and Noise')
% ylim([-1 1]);
% grid on
% subplot(3,1,2);
% plot(tspan,m2);
% ylim([-1 1]);
% grid on
% subplot(3,1,3);
% plot(tspan,m3);
% ylim([-1 1]);
% grid on

figure;
subplot(2,2,1);
[AX,H1,H2] = plotyy(tspan,xx(:,1),tspan,(xx(:,3)-pi)*(180/pi),'plot');
grid on
set(get(AX(1),'Ylabel'),'String','ball position (m)')
set(get(AX(2),'Ylabel'),'String','chassis angle (degrees)')
title('Ballbot Positions along X')

subplot(2,2,3);
[AX,H1,H2] = plotyy(tspan,xx(:,2),tspan,xx(:,4)*(180/pi),'plot');
grid on
set(get(AX(1),'Ylabel'),'String','ball velocity (m/s)')
set(get(AX(2),'Ylabel'),'String','chassis angle (degrees/s)')
title('Ballbot Velocities along X')

subplot(2,2,2);
[AX,H1,H2] = plotyy(tspan,xy(:,1),tspan,(xy(:,3)-pi)*(180/pi),'plot');
grid on
set(get(AX(1),'Ylabel'),'String','ball position (m)')
set(get(AX(2),'Ylabel'),'String','chassis angle (degrees)')
title('Ballbot Positions along Y')

subplot(2,2,4);
[AX,H1,H2] = plotyy(tspan,xy(:,2),tspan,xy(:,4)*(180/pi),'plot');
grid on
set(get(AX(1),'Ylabel'),'String','ball velocity (m/s)')
set(get(AX(2),'Ylabel'),'String','chassis angle (degrees/s)')
title('Ballbot Velocities along Y')

set(gcf,'Position',[10 1100 1100 600])
