clear all
close all

%% Initial Conditions
p0 = [0,0,0];
psi0 = 0;
theta0 = 0;
phi0 = 0;


R0 = [cos(psi0)*cos(theta0) -sin(psi0)*cos(theta0)+cos(psi0)*sin(theta0)*sin(phi0) sin(psi0)*sin(phi0)+cos(psi0)*sin(theta0)*cos(phi0); ...
      sin(psi0)*cos(theta0) cos(psi0)*cos(theta0)+sin(psi0)*sin(theta0)*sin(phi0)  -cos(psi0)*sin(phi0)+sin(psi0)*sin(theta0)*cos(phi0); ...
      -sin(theta0)  cos(theta0)*sin(phi0)  cos(theta0)*cos(phi0)];
  
  
  

