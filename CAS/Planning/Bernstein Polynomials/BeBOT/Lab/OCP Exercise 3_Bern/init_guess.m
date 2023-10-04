function x0 = init_guess(CONSTANTS)

N = CONSTANTS.N; 

x1 = ones(N+1,1);
x2 = ones(N+1,1);
u1 = ones(N+1,1);
u2 = ones(N+1,1);

x0 = [x1;x2;u1;u2];

end

