clear all



%% Parameters
p0 = [0;0;0];
psi0 = 0;
R0 = [cos(psi0) -sin(psi0) 0; sin(psi0) cos(psi0) 0; 0 0 1];


%% Control gains
kp = 1;
kR = 1;
