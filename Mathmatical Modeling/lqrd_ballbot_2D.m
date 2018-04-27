%% Discrete Ballbot LQR controls
% by Adam Woo

clear all; close all; clc;

m = 7.7;         % mass of chassis (kg)
M = .85;          % mass of ball (kg)
l = .1;         % length to center mass of chassis (m)
g = -9.81;      % gravity (m/s)
d = 1;          % damping factor
start = 0;      % x-axis initial location (m)
dest = 0;       % x-axis destination (m)
pushx = rand(1)*(pi/4)-rand(1)*(pi/4);   % initial angle from verticle (rads)
pushy = rand(1)*(pi/4)-rand(1)*(pi/4);   % initial angle from verticle (rads)
r=0;
noise=0;    % add noise if 1

t=10;                 % Simulation Duration (seconds)
Ts=0.01;          % Sampling Interval (seconds)
tspan=(0:Ts:t)';
tlen=length(tspan);

A = [0          1             0             0;
    0         -d/M         -m*g/M           0;
    0           0             0             1;
    0        -d/(M*l)   -(m+M)*g/(M*l)      0]

B = [  0;
      1/M;
       0;
     1/(M*l)]

C = [0   1   0   0;
     0   0   1   0];

D = [0;
     0];

Qs = [1 0 0 0;
    0 100 0 0;
    0 0 1 0;
    0 0 0 1];

Ql = [1 0 0 0;
    0 10 0 0;
    0 0 10 0;
    0 0 0 10];

R = .01;

K=zeros(1,size(A,1));

Ks = lqrd(A,B,Qs,R,Ts)

Kl = lqrd(A,B,Ql,R,Ts)

% L = [0.1819;
%     0.0161;
%     0.0000;
%    -0.0002]

%% Discrete simulation

xx=zeros(tlen,size(A,1));
xx(1,:) = [start; 0; pi; 0];
xy=zeros(tlen,size(A,1));
xy(1,:) = [start; 0; pi; 0];

% x_hat=zeros(tlen,size(A,1));
% x_hat(1,:) = [start; 0; pi+rand(1); 0];

yx=zeros(tlen,size(C,1));
yy=zeros(tlen,size(C,1));

% y(1,2)=pi+angle;
% y_hat=zeros(tlen,size(C,1));

motor=zeros(tlen,3);



for i = 2:tlen
    
    if noise == 1
        % noise input
        xx(i-1,3)=xx(i-1,3)+(rand(1)/(32*pi))-(rand(1)/(32*pi));
        xy(i-1,3)=xy(i-1,3)+(rand(1)/(32*pi))-(rand(1)/(32*pi));
    end
        
    % Impulse
    if i<25
        xx(i-1,3)=xx(i-1,3)+(pushx/25);
        xy(i-1,3)=xy(i-1,3)+(pushy/25);
    end
    
    % error calculation
    errorx = xx(i-1,:)'-[dest; 0; pi; 0];
    errory = xy(i-1,:)'-[dest; 0; pi; 0];
    
    % state space control
    if (abs((xx(i-1,3)-pi)*(180/pi)) > 5)
%         K=K-(K-Kl)/2;
    else
        K=K-(K-Ks)/2;
    end
    xx(i,:)=(((A-B*Ks)*(errorx))*Ts)+xx(i-1,:)';
    yx(i,:)=xx(i,:)*C';
    
    if (abs((xy(i-1,3)-pi)*(180/pi)) > 5)
%         K=K-(K-Kl)/2;
    else
        K=K-(K-Ks)/2;
    end
    xy(i,:)=(((A-B*Ks)*(errory))*Ts)+xy(i-1,:)';
    yy(i,:)=xy(i,:)*C';

    %TODO: Full State Estimator  
%     u=(-K*x_hat(i-1,:)')+r;
%     x_hat(i,:)=(((A*x_hat(i-1,:)')+(B*u))*Ts)+L*(y(i-1)-y_hat(i-1));
%     y_hat(i)=x(i,:)*C';
% 
%     x(i,:)=(((A*x(i-1,:)')+(B*u))*Ts);
%     y(i)=x(i,:)*C';

motor(i,1) = -(xx(i,2))+((xy(i,2)/-(sqrt(3))));
motor(i,2) = -(xx(i,2))+((xy(i,2)/(sqrt(3))));
motor(i,3) = xx(i,2);

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
