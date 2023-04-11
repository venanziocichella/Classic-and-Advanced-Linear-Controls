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
G = [0 0; 1 0; 0 0; 0 1];
Baug = [B G];

x0 = [0.5; 0; 0.1; 0];



%% Controller
Q = eye(4);
R = 0.1;
KLQR = lqr(A,B,Q,R);


%% Kalman-Bucy 
Qw = 0.1*eye(2);
Rw = 1;
[LKB,P,E] = lqe(A,G,C,Qw,Rw)

xhat0 = [0;0;0;0];