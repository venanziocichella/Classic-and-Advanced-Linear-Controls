close all
clear all

%% Parameters
m = 1;
l = 0.2;
g = 9.81;
r = 0.02;

%% Initial Condition
theta_0 = 1.3;
thetadot_0 = 0; 

%% State Space realization
A = [0 1 ; -g/l  -r/(m*l^2)];
B = [0 ; 1/(m*l^2)];
C = [1 0 ; 0 1]; D = 0;