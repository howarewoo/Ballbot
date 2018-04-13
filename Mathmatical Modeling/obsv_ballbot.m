clear all, close all, clc

m = 10;          % mass of chassis (kg)
M = 1;          % mass of movement system (kg)
L = .25;        % length to center mass of chassis (m)
g = -9.81;      % gravity (m/s)
d = 1;          % damping factor

s = -1; % must be -1 for stable system

%%  Which measurements are best if we omit "x" 
A = [0 1 0 0;
    0 -d/M -m*g/M 0;
    0 0 0 1;
    0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];
B = [0; 1/M; 0; s*1/(M*L)];
A = A(2:end,2:end);
B = B(2:end);
% C = [1 0 0];  % measure x
C = [0 1 0];    % measure theta
% C = [0 0 1];  % measure theta dot
obsv(A,C)

D = zeros(size(C,1),size(B,2));
sys = ss(A,B,C,D);
det(gram(sys,'o'))