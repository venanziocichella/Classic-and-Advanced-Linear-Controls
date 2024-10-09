clear all; close all;


%% Load Parameters
CONSTANTS.par1 = 0;
CONSTANTS.par2 = 0;



%% Initial Guess
xdim = 5; % this is the dimension of opt variable
x0 = zeros(xdim,1);



%% Linear Constraints and LU Bounds
A=[]; b=[]; Aeq=[]; beq=[]; lb=[]; ub=[];



%% Optimizer
options = optimoptions('fmincon',...
    'Display','iter', ...
    'Algorithm','interior-point');

tic

[x,J] = fmincon(@(x)costfun(x,CONSTANTS),x0,A,b,Aeq,beq, ...
         lb,ub,@(x)nonlcon(x,CONSTANTS),options);

toc     
     
     
     

%% Cost Function
function J = costfun(x,CONSTANTS)

% Define cost as function of x
J = 0; 

end




%% Nonlinear Constraints
function [c,ceq] = nonlcon(x,CONSTANTS)

% Define constraints as function of x
% c and ceq should be column vectors (maybe?!)
c=[];
ceq=[];

end