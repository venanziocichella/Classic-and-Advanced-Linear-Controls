clear all
close all

A = [0 1 0;0 0 1;0 -24 -10];
B = [0 ; 10 ; -80];
C = [1 0 0];

K =  [1.25 1.25 0.19];
G = [20 ; 76 ; -240];

s = tf('s');
Ks = (-K*inv(s*eye(3)-A+G*C+B*K)*G);
Ks = minreal(Ks);