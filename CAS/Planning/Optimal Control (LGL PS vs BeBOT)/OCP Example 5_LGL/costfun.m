function J = costfun(x,CONSTANTS)
%COSTFUN Summary of this function goes here
N = CONSTANTS.N; 

y = x(1:N+1);

[tnodes,w,Diff] = LGL_PS(CONSTANTS.N,1);


yprime = (y'*Diff)';


J = w'*(sqrt(1+yprime.*yprime)./sqrt(2*10*y));
end

