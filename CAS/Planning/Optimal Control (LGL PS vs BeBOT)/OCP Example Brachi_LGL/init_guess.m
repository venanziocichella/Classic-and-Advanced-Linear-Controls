function x0 = init_guess(CONSTANTS)

N = CONSTANTS.N; 

%y = linspace(0,CONSTANTS.h,N+1)';
y = ones(N+1,1);

x0 = y;

end

