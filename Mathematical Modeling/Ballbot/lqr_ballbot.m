clear all, close all, clc

m = 2;          % mass of chassis (kg)
M = 8;          % mass of movement system (kg)
L = .75;        % length to center mass of chassis (m)
g = -9.81;      % gravity (m/s)
d = 1;          % damping factor
start = 0;      % x-axis initial location (m)
dest = 0;       % x-axis destination (m)
angle = pi/8;   % initial angle from verticle (rads)

s = 1; % pendulum up (s=1)

A = [0 1 0 0;
    0 -d/M -m*g/M 0;
    0 0 0 1;
    0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];

B = [0; 1/M; 0; s*1/(M*L)];
eig(A)

Q = [1 0 0 0;
    0 1 0 0;
    0 0 10 0;
    0 0 0 100];
R = .01;

%%
det(ctrb(A,B))

%%
K = lqr(A,B,Q,R);

tspan = 0:.001:10;
if(s==-1)
    y0 = [0; 0; 0; 0];
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K*(y-[4; 0; 0; 0])),tspan,y0);
elseif(s==1)
    y0 = [start; 0; pi+angle; 0];
% % [t,y] = ode45(@(t,y)((A-B*K)*(y-[0; 0; pi; 0])),tspan,y0);
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K*(y-[dest; 0; pi; 0])),tspan,y0);
else
    
end

for k=1:100:length(t)
    drawcartpend_ballbot(y(k,:),m,M,L);
end

% function dy = pendcart(y,m,M,L,g,d,u)