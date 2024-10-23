clear all
close all


bebotFolder = fullfile(pwd, '..', 'BeBOT');
addpath(bebotFolder);



%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Bernstein approximation of smooth functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Define the following functions
T = 12;
u1 = @(t) 1./(sqrt(1+(T-t).^2));
u2 = @(t) (T-t)./(sqrt(1+(T-t).^2));
% Plot the functions in [0,T]
t = 0:0.01:T;
figure(1)
plot(t,u1(t),'LineWidth',3,'Color','k'); hold on
set(gca,'fontsize', 26);
grid on
figure(2)
plot(t,u2(t),'Linewidth',3,'Color','k'); hold on
grid on
set(gca,'fontsize', 26);

% Their derivatives are
u1dot = @(t) (T-t)./((T-t).^2+1).^(3/2);
u2dot = @(t) -1./(T^2-2*T*t+t.^2+1).^(3/2);
% Plot the derivatives in [0,T]
figure(3)
plot(t,u1dot(t),'Linewidth',3,'Color','k'); hold on
set(gca,'fontsize', 26);
grid on
figure(4)
plot(t,u2dot(t),'Linewidth',3,'Color','k'); hold on
set(gca,'fontsize', 26);
grid on

% Their integrals are
intu1 = asinh(T);
intu2 = sqrt(T^2+1)-1;




%% Compute the Bernstein Approximation of u1 and u2
%% Compute their derivatives 
%% Compute their integrals




