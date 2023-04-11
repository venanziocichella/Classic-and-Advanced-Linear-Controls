%% Parameters
J = 10.4;
l = 1.5;
L = 2;
m = 5;
M = 75;
g = 9.81;

% Initial Conditions
theta0= 0.1;
thetadot0 = 0;
psi0 = 0;
psidot0 = 0;

x0 = [theta0; psi0; thetadot0; psidot0];

A = [0 0 1 0; ...
    0 0 0 1; ...
    (3*M*g*L + 6*m*g*l)/(2*M*L^2+6*m*l^2) 0 0 0; ...
    -(3*M*g*L + 6*m*g*l)/(2*M*L^2+6*m*l^2) 0 0 0; ...
    ];
    
B = [0 ; 0 ; -6/(2*M*L^2+6*m*l^2); (6*J + 2*M*L^2 + 6*m*l^2)/(J*(2*M*L^2 + 6*m*l^2))];

C = [1 0 0 0];
D = 0;


p = [-10 -30 -50 -100];

K = place(A,B,p);

Q = [400 0 0 0 ; 0 3 0 0 ; 0 0 2 0 ; 0 0 0 1 ];
R = 0.06;

K = lqr(A,B,Q,R);



