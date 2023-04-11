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
L = place(A(2,2),A(1,2),-10);
eta0 = -L*x0(1);