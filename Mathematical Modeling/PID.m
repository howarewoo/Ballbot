M = 2.5;    % Ball mass (lbs)
m = 20;     % Bot mass (lbs)
b = .1;     % friction
I = 0.6;  
g = 32.2;   % gravity (ft/s)
l = 0.33;   % length to bot center mass (ft)
q = (M+m)*(I+m*l^2)-(m*l)^2;
s = tf('s');
P_pend = (m*l*s/q)/(s^3 + (b*(I + m*l^2))*s^2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q);


Kp = 800;
Ki = 1;
Kd = 10;
C = pid(Kp,Ki,Kd);
T = feedback(P_pend,C);
t=0:0.01:10;
impulse(T,t)
axis([0, 2.5, -0.2, 0.2]);
title({'Response of Pendulum Position to an Impulse Disturbance';'under PID Control: Kp = 100, Ki = 1, Kd = 20'});
ylabel('Position (ft)') % y-axis label

%%
d = .5;
dt = .1;
r = 1.625;
spr = 200;
rps = ((d/dt)*12)/(pi*r*2)
sps = round(rotations*spr)