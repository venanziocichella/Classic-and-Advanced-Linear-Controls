clear all
close all

CONSTANTS.N = 250; % Order of approximation

load_parameters;

x0 = init_guess(CONSTANTS);

A=[]; b=[]; Aeq=[]; beq=[]; lb=[]; ub=[]; 

options = optimoptions(@fmincon,'Algorithm','sqp')%,'MaxFunctionEvaluations',300000);
tic
[x,f] = fmincon(@(x)costfun(x,CONSTANTS),x0,A,b,Aeq,beq,lb,ub,@(x)nonlcon(x,CONSTANTS),options);
toc

x1 = x(1:end-1)';
Tf = x(end);
[tnodes,w,Dm] = BeBOT(CONSTANTS.N,Tf);
x2 = x1*Dm;
u = x2*Dm;
t = 0:0.01:Tf;
c1 = BernsteinPoly(x1,t);
c2 = BernsteinPoly(x2,t);
c3 = BernsteinPoly(u,t);
%% Plot
plot(t,c1,t,c2,t,c3)
legend('X1', 'X2', 'U')
