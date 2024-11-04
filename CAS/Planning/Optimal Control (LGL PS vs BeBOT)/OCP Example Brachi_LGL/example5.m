clear all
close all

CONSTANTS.N = 70; % Order of approximation
CONSTANTS.h = 1;

N = CONSTANTS.N; 

%y = linspace(0,CONSTANTS.h,N+1)';
y = ones(N+1,1);

x0 = y;

A=[]; b=[]; Aeq=[]; beq=[]; lb=[]; ub=[]; 

options = optimoptions(@fmincon,'Algorithm','sqp','MaxFunctionEvaluations',300000);
[x,f] = fmincon(@(x)costfun(x,CONSTANTS),x0,A,b,Aeq,beq,lb,ub,@(x)nonlcon(x,CONSTANTS),options);
















N = CONSTANTS.N;
[tnodes,w,Diff] = LGL_PS(CONSTANTS.N,1);
y = x(1:N+1);

t = 0:0.01:1;


figure
plot(tnodes,-y,'o'); hold on
plot(t,-LagrangePoly(y,tnodes,t));


function J = costfun(x,CONSTANTS)
%COSTFUN Summary of this function goes here
N = CONSTANTS.N; 

y = x(1:N+1);

[tnodes,w,Diff] = LGL_PS(CONSTANTS.N,1);


yprime = (y'*Diff)';


J = w'*(sqrt(1+yprime.*yprime)./sqrt(2*10*y));
end



function [c,ceq] = nonlcon(x,CONSTANTS)
%NONLCON Summary of this function goes here
%   Detailed explanation goes here
N = CONSTANTS.N; 

y = x(1:N+1);

c=[];
ceq=[y(1)-0; y(end) - CONSTANTS.h];
end
