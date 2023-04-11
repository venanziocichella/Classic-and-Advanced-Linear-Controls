clear all
close all
clc

%% Constant parameters
M = 75;
m = 5;
L = 2;
l = 1;
J = 10.4;
g = 9.81;


%% Initial Conditions
theta0= 0.01;
thetadot0 = 0;
psi0 = 0;
psidot0 = 0;

x0 = [theta0;thetadot0;psi0;psidot0];


%% State Space Matrices
A = [0 0 1 0; ...
    0 0 0 1; ...
    (3*M*g*L + 6*m*g*l)/(2*M*L^2+6*m*l^2) 0 0 0; ...
    -(3*M*g*L + 6*m*g*l)/(2*M*L^2+6*m*l^2) 0 0 0; ...
    ];
    
B = [0 ; 0 ; -6/(2*M*L^2+6*m*l^2); (6*J + 2*M*L^2 + 6*m*l^2)/(J*(2*M*L^2 + 6*m*l^2))];

C = [1 0 0 0];
D = 0;



%% Controllability
Co = ctrb(A,B);
rank(Co);

% Find the controller canonical form
a = charpoly(A); 
ACCF = [0 1 0 0; 0 0 1 0; 0 0 0 1; -a(5) -a(4) -a(3) -a(2)];
BCCF = [0 ; 0 ; 0 ; 1];
% We now compute the transformation TCCF
CoCCF = [BCCF ACCF*BCCF ACCF^2*BCCF ACCF^3*BCCF];
TCCF = Co*inv(CoCCF);
% Then, the matrices CCCF and DCCF are easily found
CCCF = C*TCCF;
DCCF = D;



%% Observability
% Observability matrix:
Ob = obsv(A,C);
rank(Ob) % The rank is 2, so the system is not observable


% Find the observable/unobservable modes 
% The system is not observable (2 modes are observable, 2 modes are not)
Ob' 
% Select the first two linearly independent columns
% Complete the basis
tildeT = [Ob(1,:);Ob(2,:);0 0 0 1; 0 1 0 0]';
% Find the transformation
T = inv(tildeT)';
% This gives the new system
Ahat = inv(T)*A*T;
Bhat = inv(T)*B;
Chat = C*T;
Dhat = D;



% Find the OCF
% The obsv matrix is not invertible. However, we can use duality to find
% the observer canonical form.
AOCF = ACCF';
BOCF = CCCF';
COCF = BCCF';
DOCF = DCCF;





%% Stability 
% Find if the system is stable
% Check eigenvalues of matrix A:
eig(A)

% There is a positive eigenvalue, so the system is not (internally) stable

% Find the Jordan canonical form
% The coefficients of the characteristic polynomial are
[ T,J ] = jordan(A)
% Then, the Bhat , Chat matrices are  
Bhat = inv(T)*B;
Chat = C*T;

% Find the stable,  unstable, and center (invariant) subspaces (in x-coordinates)
% First, we find the eigenvector and eigenvalues of the system
[eigV, eigv] = eig(A)
% The stable subspace is the eigenvector associated with the stable eigenvalue
Sigma_S = eigV(:,4);
% The center subspace is the eigenvector associated with the zero eigenvalue
Sigma_C = [0 ; 1 ; 0; 0];


% Find if the system is BIBO stable
% The TF of the system is 
s = tf('s')
G = C*inv(s*eye(4)-A)*B+D
% Find if the system is stable
isstable(G)
% The system is unstable





