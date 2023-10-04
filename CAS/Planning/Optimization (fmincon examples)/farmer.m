clear all; close all;


%% Load Parameters
CONSTANTS.totlength = 1000;



%% Initial Guess
x0 = zeros(2,1);



%% Linear Constraints and LU Bounds
A=[]; b=[]; Aeq=[]; beq=[]; lb=[]; ub=[];



%% Optimizer
[x,J] = fmincon(@(x)costfun(x,CONSTANTS),x0,A,b,Aeq,beq, ...
         lb,ub,@(x)nonlcon(x,CONSTANTS));
     
     
     

%% Cost Function
function J = costfun(x,CONSTANTS)
J = -x(1)*x(2);
end




%% Nonlinear Constraints
function [c,ceq] = nonlcon(x,CONSTANTS)
c=[];
ceq=[x(1)+2*x(2)-CONSTANTS.totlength];
end