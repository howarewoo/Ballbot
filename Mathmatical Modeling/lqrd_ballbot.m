%% Discrete Ballbot LQR controls
% by Adam Woo

clear all; close all; clc;

m = 10;         % mass of chassis (kg)
M = 1;          % mass of ball (kg)
L = .25;        % length to center mass of chassis (m)
g = -9.81;      % gravity (m/s)
d = 1;          % damping factor
start = 0;      % x-axis initial location (m)
dest = 0;       % x-axis destination (m)
angle = pi/8;   % initial angle from verticle (rads)

t=20;                 % Simulation Duration (seconds)
Ts=1/100;          % Sampling Interval (seconds)
tspan=(0:Ts:t)';
tlen=length(tspan);

y=zeros(tlen,4);
y(1,:) = [start; 0; pi+angle; 0];

A = [0          1             0             0;
    0         -d/M         -m*g/M           0;
    0           0             0             1;
    0        -d/(M*L)   -(m+M)*g/(M*L)      0];

B = [  0;
      1/M;
       0;
     1/(M*L)];

C = [1   0   0   0;
     0   0   1   0];

D = [0;
     0];

Q = [1 0 0 0;
    0 1 0 0;
    0 0 10 0;
    0 0 0 100];

R = .0001;

K = lqrd(A,B,Q,R,Ts)

%% Discrete simulation

push = rand(1)/40000;
pull = -rand(1)/40000;
tps =  round(rand(1)*(tlen/1.5));
tpl =  round(rand(1)*(tlen/1.5));

for i = 2:tlen
    y(i,:)=(((A-B*K)*(y(i-1,:)'-[dest; 0; pi; 0]))*Ts)+y(i-1,:)';
    
    %interference
    if(i>tps && i<tps+100)
        y(i,3)=y(i,3)-(push*i);
    end
    if(i>tpl && i<tpl+100)
        y(i,3)=y(i,3)-(pull*i);
    end
    
end

%% Draw Simulation
close all;
for k=1:1/(10*Ts):length(tspan)
    drawballbot(y(k,:),m,M,L);
end

figure;
[AX,H1,H2] = plotyy(tspan,y(:,1),tspan,y(:,3),'plot');
set(get(AX(1),'Ylabel'),'String','ball position (m)')
set(get(AX(2),'Ylabel'),'String','chassis angle (radians)')
title('Ballbot with State-Feedback Control and Interference')