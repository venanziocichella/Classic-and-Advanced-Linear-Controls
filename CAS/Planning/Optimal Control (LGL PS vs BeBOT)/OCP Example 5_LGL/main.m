clear all
close all

CONSTANTS.N = 70; % Order of approximation

load_parameters;

x0 = init_guess(CONSTANTS);

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


