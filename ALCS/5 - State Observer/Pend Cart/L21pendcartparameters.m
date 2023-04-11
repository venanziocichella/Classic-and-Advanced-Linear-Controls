clear all
close all

%% System
m = 1;
M = 7;
L = 2;
g = 10;
d = 1;


A = [0 1 0 0; 0 -d/M m*g/M 0; 0 0 0 1; 0 d/(M*L) -(m+M)*g/(M*L) 0];
B = [0 ; 1/M ; 0 ; -1/(M*L)];
C = [1 0 0 0];
D = 0;
G = ones(4,1);
Baug = [B G];

x0 = [0; 0; 0; 0];





%% Kalman-Bucy 
Qw = 0.1*eye(4);
Rw = 1;
[LKB,P,E] = lqe(A,eye(4),C,Qw,Rw)

xhat0 = [0;0;0;0];