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


%% Observer
x0hat = [x0(1); 0];
L = place(A',C',[-10 -20])';
eig(A - L*C)


%% Kalman-Bucy 
[LKB,P,E] = lqe(A,eye(2),C,0.1*ones(2,2),1)
