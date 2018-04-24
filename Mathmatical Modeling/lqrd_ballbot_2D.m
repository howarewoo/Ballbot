%% Discrete Ballbot LQR controls
% by Adam Woo

clear all; close all; clc;

m = 10;         % mass of chassis (kg)
M = 1;          % mass of ball (kg)
l = .25;        % length to center mass of chassis (m)
g = -9.81;      % gravity (m/s)
d = 1;          % damping factor
start = 0;      % x-axis initial location (m)
dest = 0;       % x-axis destination (m)
pushx = rand(1)*(pi/4);   % initial angle from verticle (rads)
pushy = rand(1)*(pi/4);   % initial angle from verticle (rads)
r=0;

t=10;                 % Simulation Duration (seconds)
Ts=1/100;          % Sampling Interval (seconds)
tspan=(0:Ts:t)';
tlen=length(tspan);

A = [0          1             0             0;
    0         -d/M         -m*g/M           0;
    0           0             0             1;
    0        -d/(M*l)   -(m+M)*g/(M*l)      0];

B = [  0;
      1/M;
       0;
     1/(M*l)];

C = [0   1   0   0;
     0   0   1   0];

D = [0;
     0];

Qs = [10 0 0 0;
    0 1 0 0;
    0 0 10 0;
    0 0 0 10];

Ql = [1 0 0 0;
    0 10 0 0;
    0 0 10 0;
    0 0 0 10];

R = .001;

Ks = lqrd(A,B,Qs,R,Ts)

Kl = lqrd(A,B,Ql,R,Ts)

% L = [0.1819;
%     0.0161;
%     0.0000;
%    -0.0002]

%% Discrete simulation

x=zeros(tlen,size(A,1));
x(1,:) = [start; 0; pi; 0];

% x_hat=zeros(tlen,size(A,1));
% x_hat(1,:) = [start; 0; pi+rand(1); 0];

y=zeros(tlen,size(C,1));
% y(1,2)=pi+angle;
% y_hat=zeros(tlen,size(C,1));

for i = 2:tlen
    
    % noise input
    xx(i-1,3)=xx(i-1,3)+rand(1)/(32*pi)-rand(1)/(32*pi);
    
    % Impulse
    if i<50
        xx(i-1,3)=xx(i-1,3)+(pushx/50);
    end
    
    % error calculation
    errorx = xx(i-1,:)'-[dest; 0; pi; 0];
    
    % state space control
    if (abs((xx(i-1,3)-pi)*(180/pi)) >= 5)
        xx(i,:)=(((A-B*Kl)*(error))*Ts)+xx(i-1,:)';
    else
        xx(i,:)=(((A-B*Ks)*(error))*Ts)+xx(i-1,:)';
    end
    yx(i,:)=xx(i,:)*C';

    %TODO: Full State Estimator  
%     u=(-K*x_hat(i-1,:)')+r;
%     x_hat(i,:)=(((A*x_hat(i-1,:)')+(B*u))*Ts)+L*(y(i-1)-y_hat(i-1));
%     y_hat(i)=x(i,:)*C';
% 
%     x(i,:)=(((A*x(i-1,:)')+(B*u))*Ts);
%     y(i)=x(i,:)*C';
    
end

%% Draw Simulation
close all;
for k=1:1/(10*Ts):length(tspan)
    drawballbot_2D(xx(k,:),m,M,l);
end

figure;
subplot(2,1,1);
[AX,H1,H2] = plotyy(tspan,xx(:,1),tspan,(xx(:,3)-pi)*(180/pi),'plot');
grid on
set(get(AX(1),'Ylabel'),'String','ball position (m)')
set(get(AX(2),'Ylabel'),'String','chassis angle (degrees)')
title('Ballbot Positions with State-Feedback Control and Noise')

subplot(2,1,2);
[AX,H1,H2] = plotyy(tspan,xx(:,2),tspan,xx(:,4)*(180/pi),'plot');
grid on
set(get(AX(1),'Ylabel'),'String','ball velocity (m/s)')
set(get(AX(2),'Ylabel'),'String','chassis angle (degrees/s)')
title('Ballbot Velocities with State-Feedback Control and Noise')