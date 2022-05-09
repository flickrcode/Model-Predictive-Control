function [Z,VN]=CRHC1_43(A,B,N,Q,R,Pf,F1,G1,h1,F2,G2,h2,x0)
%% do not chane the inputs and outputs!
%% A and B are the system matrices when x(k+1)=Ax(k)+Bu(k)
%% Q, R, and Pf are the gains in the cost function
%% N is the length of the horizon
%% Z is the vector of optimal variables and VN is the cost function 
%% F1, G1, h1, F2, G2, h2 are constraint matrices
%% Be aware of the F1, F2, G1, G2, h1, and h2! 
%% x0 is the initial condition
% Define the inputs to the quadprog function
AA = [F2 G2];
b = h2;
Aeq = [F1 G1];
beq = h1;
f = 0;

% Calculate H
H = blkdiag(Q,zeros(1,N-2),Pf,R,zeros(1,N-1))

% Calculate Z
Z = quadprog(H,f,AA,b,Aeq,beq,[],[],0)

for k = 0:N-1
   VN_1 = Z(k)'*[Q R]*Z(k)
end

% Calculate VN
X = Z(1:N)
  VN = VN_1+X(N)'*Pf*X(N)
end
