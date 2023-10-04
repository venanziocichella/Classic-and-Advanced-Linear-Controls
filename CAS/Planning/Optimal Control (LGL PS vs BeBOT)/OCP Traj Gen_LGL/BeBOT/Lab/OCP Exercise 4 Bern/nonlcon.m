function [c,ceq] = nonlcon(x,CONSTANTS)
%NONLCON Summary of this function goes here
%   Detailed explanation goes here
N = CONSTANTS.N;
x1 = zeros(1, N+1);
x1(1) = CONSTANTS.x1_initialPos;
x1(end) = CONSTANTS.x1_finalPos;
x1(2:end-1) = x(1:end-1)';
Tf = x(end);
[tnodes,w,Dm] = BeBOT(CONSTANTS.N,Tf);
x2 = x1*Dm;
u = x2*Dm;


c=[-u'-1; u'-1];
ceq=[x1(1); x2(1); x1(end)-1; x2(end)];
end

