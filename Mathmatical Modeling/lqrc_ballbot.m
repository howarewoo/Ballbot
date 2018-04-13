%% Ballbot LQR controls
% by Adam Woo

clear all, close all, clc

m = 10;          % mass of chassis (kg)
M = 1;          % mass of ball (kg)
L = .25;        % length to center mass of chassis (m)
g = -9.81;      % gravity (m/s)
d = 1;          % damping factor
start = 0;      % x-axis initial location (m)
dest = 0;       % x-axis destination (m)
angle = pi/8;  % initial angle from verticle (rads)

s = 1; % pendulum up (s=1)

Ts=1/1000;
tspan = (0:Ts:10)';
tlen=length(tspan);

y(1,:) = [start; 0; pi+angle; 0];

A = [0              1               0               0;
    0             -d/M           -m*g/M             0;
    0               0               0               1;
    0           -s*d/(M*L)   -s*(m+M)*g/(M*L)       0]

B = [    0;
        1/M;
         0;
     s*1/(M*L)] 

C = [1   0   0   0;
     0   0   1   0];

D = [0;
     0];

eig(A);

Q = [1 0 0 0;
    0 1 0 0;
    0 0 10 0;
    0 0 0 100];

R = .0001;

det(ctrb(A,B))

K = lqr(A,B,Q,R)

%%
[t,y] = ode45(@(t,y)((A-B*K)*(y-[dest; 0; pi; 0])),tspan,y(1,:));
% [t,y] = ode45(@(t,y)ballbot(y,m,M,L,g,d,-K*(y-[dest; 0; pi; 0])),tspan,y(1,:));

for k=1:100:length(t)
    drawballbot(y(k,:),m,M,L);
end