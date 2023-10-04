clear all
close all

%% Initial Conditions
p0 = [0,0];
psi0 = 0;


R0 = [cos(psi0) -sin(psi0); ...
      sin(psi0) cos(psi0)]; 
  
  
  
  
%% Trajectory tracking parameters
KP = 1;
KR = 1;