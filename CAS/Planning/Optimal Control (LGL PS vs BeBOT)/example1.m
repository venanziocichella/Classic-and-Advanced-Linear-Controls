clear all
close all

%% load parameters
CONSTANTS.N = 5;
CONSTANTS.x0 = 3;
CONSTANTS.xf = 0;
CONSTANTS.vmax = 5;
CONSTANTS.vmin = -5;

%% initial guess
xinit = [ones(2*CONSTANTS.N+2,1); 5];

%% linear constraints and UL bounds
A = []; b = []; Aeq = []; beq = []; lb = []; ub = [];


%% optimizer
[x,J] = fmincon(@(x)costfun(x,CONSTANTS),xinit,A,b,Aeq,beq,lb,ub,@(x)nonlcon(x,CONSTANTS));



%% plot
x1 = x(1:CONSTANTS.N+1);
u1 = x(CONSTANTS.N+2:end-1);
tf = x(end);

[tnodes,w,Diff] = LGL_PS(CONSTANTS.N,tf);

t = 0:0.01:tf;
x1N = LagrangePoly(x1,tnodes,t);
u1N = LagrangePoly(u1,tnodes,t);

figure
plot(tnodes,x1); hold on
plot(t,x1N);

figure
plot(tnodes,u1,'o'); hold on
plot(t,u1N);




%% cost function
function J = costfun(x,CONSTANTS)
    x1 = x(1:CONSTANTS.N+1);
    u1 = x(CONSTANTS.N+2:end-1);
    tf = x(end);
    
    J = tf;
end

%% nonlinear constraints
function [c, ceq] = nonlcon(x,CONSTANTS)

    x1 = x(1:CONSTANTS.N+1);
    u1 = x(CONSTANTS.N+2:end-1);
    tf = x(end);
    
    [tnodes,w,Diff] = LGL_PS(CONSTANTS.N,tf);
    x1dot = (x1'*Diff)';
    dyn = x1dot - u1;
    
    nonlcon1 = u1 - CONSTANTS.vmax;
    nonlcon2 = -u1 + CONSTANTS.vmin;
    
    IC = x1(1) - CONSTANTS.x0;
    FC = x1(end) - CONSTANTS.xf;
    
    c = [nonlcon1 ; nonlcon2; -tf];
    ceq = [dyn ; IC ; FC];
    
end