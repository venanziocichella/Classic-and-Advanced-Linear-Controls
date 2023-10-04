function J = costfun(x,CONSTANTS)
%COSTFUN Summary of this function goes here
N = CONSTANTS.N; 

x1 = x(1:N+1);
x2 = x(N+2:2*N+2);
u1 = x(2*N+3:3*N+3);
u2 = x(3*N+4:end);

J = -x1(end);
end

