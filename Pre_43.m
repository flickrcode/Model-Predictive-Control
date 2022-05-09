function P=Pre_43(A,B,S,U)
% A and B are the system matrices x^+=Ax+Bu
% S is the polytope for set S
% U is the polytope for feasible inputs
% P is the polytope Pre(S)

Hs = S.A;
hs = S.b;
Hu = U.A;
hu = U.b;
PH = [Hs*A Hs*B ; zeros(size(Hu,1),size(Hs*A,2)) Hu];
Ph = [hs;hu];
ProjPoly = Polyhedron(PH,Ph);

P = projection(ProjPoly,[1;2]);
end