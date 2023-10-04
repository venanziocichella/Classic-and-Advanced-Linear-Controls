function [c,ceq] = nonlcon(x,CONSTANTS)
%NONLCON Summary of this function goes here
%   Detailed explanation goes here
N = CONSTANTS.N; 

x1 = x(1:N+1);
x2 = x(N+2:2*N+2);
u1 = x(2*N+3:3*N+3);
u2 = x(3*N+4:end);

[tnodes,w,Diff] = BeBOT(CONSTANTS.N,CONSTANTS.T);

dyn1 = x1'*Diff-x2'-u1';
dyn2 = x2'*Diff-u2';

nonlcon1 = u1.^2+u2.^2 - 1;


c=[];
ceq=[dyn1'; dyn2'; nonlcon1;x1(1);x2(1)];
end

