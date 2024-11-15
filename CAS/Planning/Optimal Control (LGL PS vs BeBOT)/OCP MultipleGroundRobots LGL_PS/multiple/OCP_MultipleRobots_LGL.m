close all hidden
clear
clc

%% Load parameters
CONSTANTS.N = 5; % Order of approximation (low is ok due to useful properties of Bernstein polynomials)
CONSTANTS.pinit = [ % Initial points for both vehicles
    0, 0;
    10, 0];
CONSTANTS.pfin = [  % Final points for both vehicles
    10, 10;
    0, 10];
CONSTANTS.pobst = [ % Obstacle's position
     5;
     5];

CONSTANTS.vmin = 0.1;   % Minimum speed
CONSTANTS.vmax = 1;     % Maximum speed
CONSTANTS.omegamax = 1;   % Maximum angular rate
CONSTANTS.mindist = 1;      % Minimum safe distance
CONSTANTS.Nveh = 2;     % Number of vehicles
CONSTANTS.elev = 10;    % Amount by which to elevate the degree when checking constraints







%% Initial guess
N = CONSTANTS.N; 
Nveh = CONSTANTS.Nveh;
px = [linspace(CONSTANTS.pinit(1, 1), CONSTANTS.pfin(1, 1), N+1), linspace(CONSTANTS.pinit(2, 1), CONSTANTS.pfin(2, 1), N+1)];
py = [linspace(CONSTANTS.pinit(1, 2), CONSTANTS.pfin(1, 2), N+1), linspace(CONSTANTS.pinit(2, 2), CONSTANTS.pfin(2, 2), N+1)];
psi = [pi/4*ones(1,N+1), -pi/4*ones(1,N+1)];
V = 4*ones(1,2*(N+1));
omega = 1*ones(1,2*(N+1));
T = 10;
x0 = [px'; py'; psi'; V'; omega'; T];





%% UL Bounds
% These are our linear constraints and bounds. There is at least one
% variable that can be given a lower bound (without bounds, the variables
% are assumed to be on (-inf, inf))
A=[]; b=[]; Aeq=[]; beq=[]; lb=[]; ub=[]; 

% options = optimoptions(@fmincon, ...
%     'MaxIter', 1000000, ...     Maximum number of iterations
%     'MaxFunEvals', 1000000, ...   Maximum number of function evaluations
%     'Display', 'iter-detailed', ... Display the progress
%     'DiffMinChange', 1e-3, ...  Similar to the relaxation bound of the problem
%     'Algorithm', 'SQP' ...  We are using the SQP algorithm to solve this OCP
%     );
options = optimoptions(@fmincon,'Algorithm','sqp','MaxFunctionEvaluations',3000000);

[x, f] = fmincon(@(x) costfun(x,CONSTANTS), x0, A, b, Aeq, beq, lb, ub, ...
                 @(x) nonlcon(x,CONSTANTS), options);

% Reshape the optimized value so we can visualize it

N = CONSTANTS.N;
Nveh = CONSTANTS.Nveh;
T = x(end);

clear px py psi V omega
px(1,:) = x(1:N+1);
px(2,:) = x(N+2:2*N+2);
py(1,:) = x(2*N+3:3*N+3);
py(2,:) = x(3*N+4:4*N+4);
psi(1,:) = x(4*N+5:5*N+5);
psi(2,:) = x(5*N+6:6*N+6);
V(1,:) = x(6*N+7:7*N+7);
V(2,:) = x(7*N+8:8*N+8);
omega(1,:) = x(8*N+9:9*N+9);
omega(2,:) = x(9*N+10:10*N+10);

[tnodes, ~, ~] = LGL_PS(N,T);
t = 0:0.01:T;

% Create the trajectories for vehicle 1 and vehicle 2
c1 = [LagrangePoly(px(1,:),tnodes,t); LagrangePoly(py(1,:),tnodes,t)];
c2 = [LagrangePoly(px(2,:),tnodes,t); LagrangePoly(py(2,:),tnodes,t)];


% Plot the trajectories
figure(1)
plot(c1(1, :), c1(2, :)), hold on
plot(c2(1, :), c2(2, :))
pos = [CONSTANTS.pobst(1)-CONSTANTS.mindist CONSTANTS.pobst(2)-CONSTANTS.mindist 2*CONSTANTS.mindist 2*CONSTANTS.mindist]; 
%pos = [1 2 4 4]; 
rectangle('Position',pos,'Curvature',[1 1])
axis equal

% Plot the velocities
figure(2)
plot(t, LagrangePoly(V(1,:),tnodes,t)); hold on
plot(t, LagrangePoly(V(2,:),tnodes,t));
grid on

% Plot the ang velocities
figure(3)
plot(t, LagrangePoly(omega(1,:),tnodes,t)); hold on
plot(t, LagrangePoly(omega(2,:),tnodes,t));
grid on

% The following code animates the vehicles following the trajectories. This
% can be useful since it is difficult to tell whether the trajectories
% indeed guarantee that the vehicles do not collide at any point in time.
% m1 = plot(c1(1, 1), c1(2, 1), 'o', 'MarkerSize', 10);
% m2 = plot(c2(1, 1), c2(2, 1), 'o', 'MarkerSize', 10);
% for i = 1:length(t)
%     m1.XData = c1(1, i)
%     m1.YData = c1(2, i);
%     m2.XData = c2(1, i);
%     m2.YData = c2(2, i);
%     pause(0.01)
% end










%% Cost function
function J = costfun(x, CONSTANTS)
%COSTFUN Returns the current cost given the vector X
% This cost function simply returns the last element in the vector X, which
% is the final time.
    
    J = x(end);
end




%% Nonlinear constraints
function [c, ceq] = nonlcon(x, CONSTANTS)
%NONLCON Nonlinear constraints function for the 2D, 2 Dubin's car example
%   The nonlinear constraints for this function include min and max speed,
%   max angular rate, and minimum safe distance between vehicles.
x = x';
N = CONSTANTS.N;
Nveh = CONSTANTS.Nveh;
T = x(end);

px(1,:) = x(1:N+1);
px(2,:) = x(N+2:2*N+2);
py(1,:) = x(2*N+3:3*N+3);
py(2,:) = x(3*N+4:4*N+4);
psi(1,:) = x(4*N+5:5*N+5);
psi(2,:) = x(5*N+6:6*N+6);
V(1,:) = x(6*N+7:7*N+7);
V(2,:) = x(7*N+8:8*N+8);
omega(1,:) = x(8*N+9:9*N+9);
omega(2,:) = x(9*N+10:10*N+10);

[~, ~, Diff] = LGL_PS(CONSTANTS.N,T);

dyn1 = px(1,:)*Diff - V(1,:).*cos(psi(1,:));
dyn2 = py(1,:)*Diff - V(1,:).*sin(psi(1,:));
dyn3 = psi(1,:)*Diff - omega(1,:);
dyn4 = px(2,:)*Diff - V(2,:).*cos(psi(2,:));
dyn5 = py(2,:)*Diff - V(2,:).*sin(psi(2,:));
dyn6 = psi(2,:)*Diff - omega(2,:);


% Compute the degree elevation matrix. Like the differentiation matrix,
% this can be computed ahead of time to significantly improve the
% computational efficiency.
speed = [V(1,:), V(2,:)];
angrate = [omega(1,:), omega(2,:)];

% Minimum distance squared between two vehicles is computed almost
% identically to that of the speed.
dist = (px(1,:)-px(2,:), px(1,:)-px(2,:)) + BernsteinProduct(py(1,:)-py(2,:), py(1,:)-py(2,:));

% make sure that our resulting values are less than or equal to zero (not
% greater than or equal to zero) due to the way fmincon handles the
% inequality constraints. Don't forget to square the values that need to be
% squared.
c = [
%     psi(1,:)' - 2*pi;
%     - psi(1,:)'; 
%     psi(2,:)' - 2*pi;
%     - psi(2,:)';
    speed' - CONSTANTS.vmax;
    -speed' + CONSTANTS.vmin;
    angrate' - CONSTANTS.omegamax;
    -angrate' - CONSTANTS.omegamax;
    ];

% Equality constrains: dynamics and IC/FC conditions
ceq = [
    dyn1';
    dyn2';
    dyn3';
    dyn4';
    dyn5';
    dyn6';
    px(1,1)-CONSTANTS.pinit(1,1);
    py(1,1)-CONSTANTS.pinit(1,2);
    px(2,1)-CONSTANTS.pinit(2,1);
    py(2,1)-CONSTANTS.pinit(2,2);
    px(1,end)-CONSTANTS.pfin(1,1);
    py(1,end)-CONSTANTS.pfin(1,2);
    px(2,end)-CONSTANTS.pfin(2,1);
    py(2,end)-CONSTANTS.pfin(2,2););
    ];


end
    