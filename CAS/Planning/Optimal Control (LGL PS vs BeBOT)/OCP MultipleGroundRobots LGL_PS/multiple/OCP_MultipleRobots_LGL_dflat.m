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
CONSTANTS.omegamax = 0.5;   % Maximum angular rate
CONSTANTS.mindist = 1;      % Minimum safe distance

CONSTANTS.Nveh = 2;     % Number of vehicles
CONSTANTS.elev = 10;    % Amount by which to elevate the degree when checking constraints







%% Initial guess
N = CONSTANTS.N; 

x1 = linspace(CONSTANTS.pinit(1, 1), CONSTANTS.pfin(1, 1), N+1)';
y1 = linspace(CONSTANTS.pinit(1, 2), CONSTANTS.pfin(1, 2), N+1)';
x2 = linspace(CONSTANTS.pinit(2, 1), CONSTANTS.pfin(2, 1), N+1)';
y2 = linspace(CONSTANTS.pinit(2, 2), CONSTANTS.pfin(2, 2), N+1)';
T = 10;

x0 = [
    x1(2:end-1);
    y1(2:end-1);
    x2(2:end-1);
    y2(2:end-1);
    T];





%% UL Bounds
% These are our linear constraints and bounds. There is at least one
% variable that can be given a lower bound (without bounds, the variables
% are assumed to be on (-inf, inf))
A=[]; b=[]; Aeq=[]; beq=[]; lb=[]; ub=[]; 

options = optimoptions(@fmincon, ...
    'MaxIter', 100, ...     Maximum number of iterations
    'MaxFunEvals', 10000, ...   Maximum number of function evaluations
    'Display', 'iter-detailed', ... Display the progress
    'DiffMinChange', 1e-3, ...  Similar to the relaxation bound of the problem
    'Algorithm', 'SQP' ...  We are using the SQP algorithm to solve this OCP
    );

[x, f] = fmincon(@(x) costfun(x,CONSTANTS), x0, A, b, Aeq, beq, lb, ub, ...
                 @(x) nonlcon(x,CONSTANTS), options);

% Reshape the optimized value so we can visualize it
N = CONSTANTS.N;
T = x(end);
nrow = CONSTANTS.Nveh*2;
ncol = CONSTANTS.N + 1;
y = zeros(nrow, ncol);
y(:, 1) = reshape(CONSTANTS.pinit', [], 1);
y(:, end) = reshape(CONSTANTS.pfin', [], 1);
y(:, 2:end-1) = reshape(x(1:end-1), ncol-2, nrow)';
x1 = y(1, :);
y1 = y(2, :);
x2 = y(3, :);
y2 = y(4, :);
t = 0:0.01:T;

% Create the trajectories for vehicle 1 and vehicle 2
c1 = [BernsteinPoly(x1, t); BernsteinPoly(y1, t)];
c2 = [BernsteinPoly(x2, t); BernsteinPoly(y2, t)];

% Plot speed and angrate
% Create the differentiation matrix (note that this can be done ahead of
% time for improved computational efficiency)
[tnodes, w, Diff] = BeBOT(N, T);

% Find the velocity and acceleration in each dimension for each vehicle. We
% can use differential flatness for our constraints rather than using an
% equality constraint on the vehicle dynamics.
x1dot = x1*Diff;
y1dot = y1*Diff;
x2dot = x2*Diff;
y2dot = y2*Diff;
x1ddot = x1dot*Diff;
y1ddot = y1dot*Diff;
x2ddot = x2dot*Diff;
y2ddot = y2dot*Diff;

speed1 = BernsteinProduct(x1dot, x1dot) + BernsteinProduct(y1dot, y1dot);
speed2 = BernsteinProduct(x2dot, x2dot) + BernsteinProduct(y2dot, y2dot);

num1 = BernsteinProduct(x1ddot, y1dot) - BernsteinProduct(x1dot, y1ddot);
den1 = BernsteinProduct(x1dot, x1dot) + BernsteinProduct(y1dot, y1dot);
angrate1 = (num1) ./ (den1);
num2 = BernsteinProduct(x2ddot, y2dot) - BernsteinProduct(x2dot, y2ddot);
den2 = BernsteinProduct(x2dot, x2dot) + BernsteinProduct(y2dot, y2dot);
angrate2 = (num2) ./ (den2);

figure(1)
plot(t,BernsteinPoly(speed1,t),'-'); hold on
plot(t,BernsteinPoly(speed2,t));



% Plot the trajectories
figure(3)
plot(c1(1, :), c1(2, :)), hold on
plot(c2(1, :), c2(2, :))
pos = [CONSTANTS.pobst(1)-CONSTANTS.mindist CONSTANTS.pobst(2)-CONSTANTS.mindist 2*CONSTANTS.mindist 2*CONSTANTS.mindist]; 
%pos = [1 2 4 4]; 
rectangle('Position',pos,'Curvature',[1 1])
axis equal

% The following code animates the vehicles following the trajectories. This
% can be useful since it is difficult to tell whether the trajectories
% indeed guarantee that the vehicles do not collide at any point in time.
m1 = plot(c1(1, 1), c1(2, 1), 'o', 'MarkerSize', 10);
m2 = plot(c2(1, 1), c2(2, 1), 'o', 'MarkerSize', 10);
for i = 1:length(t)
    m1.XData = c1(1, i)
    m1.YData = c1(2, i);
    m2.XData = c2(1, i);
    m2.YData = c2(2, i);
    pause(0.01)
end










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
N = CONSTANTS.N; 

% Reshape the X vector into a shape that is easier to handle

T = x(end);
nrow = CONSTANTS.Nveh*2;
ncol = CONSTANTS.N + 1;
y = zeros(nrow, ncol);
y(:, 1) = reshape(CONSTANTS.pinit', [], 1);
y(:, end) = reshape(CONSTANTS.pfin', [], 1);
y(:, 2:end-1) = reshape(x(1:end-1), ncol-2, nrow)';
x1 = y(1, :);
y1 = y(2, :);
x2 = y(3, :);
y2 = y(4, :);

% Create the differentiation matrix (note that this can be done ahead of
% time for improved computational efficiency)
[tnodes, w, Diff] = BeBOT(N, T);

% Find the velocity and acceleration in each dimension for each vehicle. We
% can use differential flatness for our constraints rather than using an
% equality constraint on the vehicle dynamics.
x1dot = x1*Diff;
y1dot = y1*Diff;
x2dot = x2*Diff;
y2dot = y2*Diff;
x1ddot = x1dot*Diff;
y1ddot = y1dot*Diff;
x2ddot = x2dot*Diff;
y2ddot = y2dot*Diff;

% Compute the degree elevation matrix. Like the differentiation matrix,
% this can be computed ahead of time to significantly improve the
% computational efficiency.
E = DegElevMatrix(2*N, 2*N+CONSTANTS.elev);

% Compute the speed squared of each vehicle. We have to find the speed
% squared rather than the speed because the norm is not defined for a
% Bernstein polynomial since we cannot take the square root. However, the
% product and sum of polynomials does result in a new Bernstein polynomial.
speed1 = BernsteinProduct(x1dot, x1dot) + BernsteinProduct(y1dot, y1dot);
speed2 = BernsteinProduct(x2dot, x2dot) + BernsteinProduct(y2dot, y2dot);
speed = [speed1*E, speed2*E];

% Compute the angular rate of both vehicles. Like the speed, it must be the
% angular rate squared. Note that the resulting angular rate is a rational
% Bernstein polynomial which only exhibits SOME of the properties of normal
% Bernstein polynomials. In this situation, we are taking advantage of the
% fact that a rational Bernstein polynomial exhibits the convex hull
% property. To have a more accurate angular rate, we elevate the degree of
% the numerator and denominator Bernstein polynomials before dividing. The
% division we are performing here just finds the control points and not the
% weights of the rational Bernstein polynomials since we only care about
% the control points.
num1 = BernsteinProduct(x1ddot, y1dot) - BernsteinProduct(x1dot, y1ddot);
den1 = BernsteinProduct(x1dot, x1dot) + BernsteinProduct(y1dot, y1dot);
angrate1 = (num1*E) ./ (den1*E);
num2 = BernsteinProduct(x2ddot, y2dot) - BernsteinProduct(x2dot, y2ddot);
den2 = BernsteinProduct(x2dot, x2dot) + BernsteinProduct(y2dot, y2dot);
angrate2 = (num2*E) ./ (den2*E);
angrate = [angrate1, angrate2];

% Minimum distance squared between two vehicles is computed almost
% identically to that of the speed.
dist = BernsteinProduct(x1-x2, x1-x2) + BernsteinProduct(y1-y2, y1-y2);
distobst1 = BernsteinProduct(x1-CONSTANTS.pobst(1), x1-CONSTANTS.pobst(1)) + BernsteinProduct(y1-CONSTANTS.pobst(2), y1-CONSTANTS.pobst(2));
distobst2 = BernsteinProduct(x2-CONSTANTS.pobst(1), x2-CONSTANTS.pobst(1)) + BernsteinProduct(y2-CONSTANTS.pobst(2), y2-CONSTANTS.pobst(2));

% make sure that our resulting values are less than or equal to zero (not
% greater than or equal to zero) due to the way fmincon handles the
% inequality constraints. Don't forget to square the values that need to be
% squared.
c = [
    speed' - CONSTANTS.vmax^2;
    -speed' + CONSTANTS.vmin^2;
    angrate' - CONSTANTS.omegamax^2;
    -angrate' - CONSTANTS.omegamax^2;
    -dist' + CONSTANTS.mindist^2;
    -distobst1' + CONSTANTS.mindist^2;
    -distobst2' + CONSTANTS.mindist^2];

% No need for the equality constraints on the initial and final positions
% because of the Reshape_Vector function.
% ceq = [
%     x1(1)-CONSTANTS.pinit(1);
%     x2(1)-CONSTANTS.pinit(2);
%     x1(end)-CONSTANTS.pfin(1);
%     x2(end)-CONSTANTS.pfin(2)];

% No equality constraints on the dynamics because the system is
% differentially flat.
ceq = [];
end
    