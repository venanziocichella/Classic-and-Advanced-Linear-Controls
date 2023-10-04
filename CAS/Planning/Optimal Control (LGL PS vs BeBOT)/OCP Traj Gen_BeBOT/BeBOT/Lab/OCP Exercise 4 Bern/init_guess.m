function x0 = init_guess(CONSTANTS)
[tnodes,w,Dm] = BeBOT(CONSTANTS.N,1);
temp = tnodes;
x0 = [temp 1]';


end

