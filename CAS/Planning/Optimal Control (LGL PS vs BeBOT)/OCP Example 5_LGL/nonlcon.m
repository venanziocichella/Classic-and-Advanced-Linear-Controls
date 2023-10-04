function [c,ceq] = nonlcon(x,CONSTANTS)
%NONLCON Summary of this function goes here
%   Detailed explanation goes here
N = CONSTANTS.N; 

y = x(1:N+1);

c=[];
ceq=[y(1)-0; y(end) - CONSTANTS.h];
end

