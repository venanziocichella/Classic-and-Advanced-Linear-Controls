clear all
close all

bebotFolder = fullfile(pwd, '..', 'BeBOT');
addpath(bebotFolder);


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Bernstein Approximation of non-smooth functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Define the following function
syms u(t)
T = 2;
u(t) =@(t) piecewise(t<=T/2, 1, T/2<t, -1);
t = 0:0.01:T;
uval = u(t);
figure(7)
plot(t,uval,'Linewidth',2,'Color','k'); hold on
set(gca,'fontsize', 26);
grid on




%% Compute the Bernstein approximation of u1 and u2
%% Compute their derivatives 
%% Compute their integrals
% HINT: Start by computing the tnodes, differentiation matrix, and
% integration weights.











%% Compute the Lagrange approximation at LGL nodes of u1 and u2
%% Compute their derivatives 
%% Compute their integrals
% HINT: Start by computing the tnodes, differentiation matrix, and
% integration weights.


