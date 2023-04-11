clear all
close all

%% System
m = 2;
l = 0.6;
r = 0.1;
g = 10;

A = [0 1 ; g/l -r/(m*l^2)];
B = [0 ; 1/(m*l^2)];
C = [1 0];


%% Initial Conditions
theta0 = 0.2;
thetadot0 = 0.04;
x0 = [theta0 ; thetadot0];


%% Observer
x0hat = [x0(1); 0];
L = place(A',C',[-15 -20])';
eig(A - L*C)