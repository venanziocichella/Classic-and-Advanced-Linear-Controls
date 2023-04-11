%% Pendulum Sim

m = 2;
l = 0.6;
r = 0.1;
g = 10;

A = [0 1 ; g/l -r/(m*l^2)];
B = [0 ; 1/(m*l^2)];



%% Initial Conditions
theta0 = 0.2;
thetadot0 = 0.04;
x0 = [theta0 ; thetadot0];


%% Controller's parameters
p1 = -6 + 6i;
p2 = -6 - 6i;
p = [p1 p2];
K = place(A,B,p);


Q = [100 0 ; 0 0.1];
R = 0.1;
K = lqr(A,B,Q,R)


%% Closed loop system
ACL = A-B*K;
eig(ACL)