clear all
close all

A = [1 1 ; 0 1]; 
B = [0 ; 1];
Bw = [1; 1];
C = [1 0];
D = 0;

x0 = [0.2; 0.75];

q = 1;
Q = q*[1 1; 1 1];
R = 1;
sigma = 10;
Qw = sigma*1;
Rw = 1;

K = lqr(A,B,Q,R);
G = lqe(A,Bw,C,Qw,Rw);

xhat0 = [0; 0];