clear all; close all;

if max(size(gcp)) == 0 % parallel pool needed
    parpool % create the parallel pool
end


%% Load Parameters
CONSTANTS.N = 10;
N = CONSTANTS.N - 1;
CONSTANTS.Diff = -[eye(N); zeros(1,N)]+[zeros(1,N);eye(N)];
CONSTANTS.distance = 2;
CONSTANTS.obstacle = [4 4];
CONSTANTS.pinit = [0 0];
CONSTANTS.pfin = [10 10];


%% Initial Guess
x0 = [linspace(CONSTANTS.pinit(1),CONSTANTS.pfin(1),CONSTANTS.N) linspace(CONSTANTS.pinit(2),CONSTANTS.pfin(2),CONSTANTS.N)]';



%% Linear Constraints and LU Bounds
A=[]; b=[]; Aeq=[]; beq=[]; lb=[]; ub=[];




%% Optimizer
options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
tic
[x,J] = fmincon(@(x)costfun(x,CONSTANTS),x0,A,b,Aeq,beq, ...
         lb,ub,@(x)nonlcon(x,CONSTANTS),options);
toc     
     
     
     
     
%% Plot 
px = x(1:CONSTANTS.N);
py = x(CONSTANTS.N+1:end);
figure(1)
plot(px,py,'o','LineWidth',2.5)
hold on
plot(px,py,'Color','g')
r = CONSTANTS.distance;
d = r*2;
pxob = CONSTANTS.obstacle(1)-r;
pyob = CONSTANTS.obstacle(2)-r;
h = rectangle('Position',[pxob pyob d d],'Curvature',[1,1]);
daspect([1,1,1])





%% Optimizer PARALLEL
options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
options = optimoptions(options,'UseParallel',true);
tic
[x,J] = fmincon(@(x)costfun(x,CONSTANTS),x0,A,b,Aeq,beq, ...
         lb,ub,@(x)nonlcon(x,CONSTANTS),options);
toc     
     
     
     
     
%% Plot 
px = x(1:CONSTANTS.N);
py = x(CONSTANTS.N+1:end);
figure(2)
plot(px,py,'o','LineWidth',2.5)
hold on
plot(px,py,'Color','g')
r = CONSTANTS.distance;
d = r*2;
pxob = CONSTANTS.obstacle(1)-r;
pyob = CONSTANTS.obstacle(2)-r;
h = rectangle('Position',[pxob pyob d d],'Curvature',[1,1]);
daspect([1,1,1])
     
     
     

%% Cost Function
function J = costfun(x,CONSTANTS)
px = x(1:CONSTANTS.N);
py = x(CONSTANTS.N+1:end);
p = [px' ; py']';

dist = vecnorm([(px'*CONSTANTS.Diff); (py'*CONSTANTS.Diff)]);
totdist = sum(dist);

J = totdist;
end




%% Nonlinear Constraints
function [c,ceq] = nonlcon(x,CONSTANTS)
px = x(1:CONSTANTS.N);
py = x(CONSTANTS.N+1:end);
p = [px' ; py']';

dist = vecnorm([(px'*CONSTANTS.Diff); (py'*CONSTANTS.Diff)]);
dist2obst = vecnorm((p - CONSTANTS.obstacle)');

c=[-dist2obst'+CONSTANTS.distance];

ceq=[x(1)- CONSTANTS.pinit(1); ...
    x(CONSTANTS.N+1) - CONSTANTS.pinit(2); ...
    x(CONSTANTS.N) - CONSTANTS.pfin(1); ...
    x(end) - CONSTANTS.pfin(2); ...
    dist'-dist(1)*ones(length(dist),1);];
end