function P=Reach_43(A,B,V,U)
% A and B are the system matrices x^+=Ax+Bu
% S is the polytope for set S
% U is the polytope for feasible inputs
% P is the polytope Reach(S)
X = A*V;
Y = B*U;

P = plus(X,Y);


end