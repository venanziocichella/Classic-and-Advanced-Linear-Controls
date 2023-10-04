clear all

CONSTANTS.N = 20; % Order of approximation

load_parameters;

x0 = init_guess(CONSTANTS);

A=[]; b=[]; Aeq=[]; beq=[]; lb=[]; ub=[]; 

options = optimoptions(@fmincon,'Algorithm','sqp','MaxFunctionEvaluations',300000);
[x,f] = fmincon(@(x)costfun(x,CONSTANTS),x0,A,b,Aeq,beq,lb,ub,@(x)nonlcon(x,CONSTANTS),options);
















N = CONSTANTS.N;

x1 = x(1:N+1);
x2 = x(N+2:2*N+2);
u1 = x(2*N+3:3*N+3);
u2 = x(3*N+4:end);

[tnodes,w,Diff] = BeBOT(CONSTANTS.N,CONSTANTS.T);
t = 0:0.001:CONSTANTS.T;

figure(4)
plot(tnodes,u1,'o'); hold on
plot(tnodes,u2,'o')
plot(t,BernsteinPoly(u1',t));
plot(t,BernsteinPoly(u2',t));
