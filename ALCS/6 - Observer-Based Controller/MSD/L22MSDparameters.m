clear all
close all

%% System
m = 2;
b = 1;
k = 2;

A = [0 1 ; -k/m -b/m];
B = [0 ; 1/m];
C = [1 0];
D = 0;

x0 = [1; 0.5];


%% LQR
Q = eye(2);
R = 0.1;
KLQR = lqr(A,B,Q,R)


%% Kalman-Bucy 
x0hat = [x0(1); 0];
[LKB,P,E] = lqe(A,eye(2),C,0.1*ones(2,2),1)
