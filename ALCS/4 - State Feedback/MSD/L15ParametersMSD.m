%% Pendulum Sim

m = 2;
b = 0.0025;
k = 2;


A = [0 1 ; -k/m -b/m];
B = [0 ; 1/m];



%% Initial Conditions
p0 = 0.4;
v0 = -0.1;
x0 = [p0 ; v0];


%% Controller's parameters
k1 = 0;
k2 = 20;
K = [k1 k2];


%% Closed loop system
ACL = A-B*K;
eig(ACL)