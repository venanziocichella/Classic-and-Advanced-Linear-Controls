clear all
close all

xn = [1 3 2 5 6 10 14 1 4 -2 1 1 2 1 3 1 1 1 2];
N = length(xn) - 1;
t = 0:0.01:1;
tnodes = linspace(0,1,N+1)
plot(tnodes,xn,'o'); hold on
plot(t,BernsteinPoly(xn,t)); 

% xnplus = xn*DegElevMatrix(N,N+1);
% tnodes = linspace(0,1,N+2)
% plot(tnodes,xnplus,'o'); hold on
% plot(t,BernsteinPoly(xnplus,t)); 
% 
% xnplusminus = DegReduction(xnplus,N);
% tnodes = linspace(0,1,N+1)
% plot(tnodes,xnplusminus,'o'); hold on
% plot(t,BernsteinPoly(xnplusminus,t)); 

xnminus = DegReduction(xn,N-1);
tnodes = linspace(0,1,N)
plot(tnodes,xnminus,'o'); hold on
plot(t,BernsteinPoly(xnminus,t)); 

legend('1','2','3','4','5','6','7','8')