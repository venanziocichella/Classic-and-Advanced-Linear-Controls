clear all
close all

x0 = [0.1 ; 0];

A = [0 1 ; 20.6 0];
B = [0 ; 1];
C = [1 1];

x0hat = [0 ; 0];
K = place(A,B,[-1.8+2.4i; -1.8-2.4i]);
G = acker(A',C',[-8; -8])';