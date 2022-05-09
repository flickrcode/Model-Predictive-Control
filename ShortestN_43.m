function [Z,Aineq,bineq,exitflag]=ShortestN_43(A,B,N,Q,R,Pf,x_ub,u_ub,x0)
%% A and B are the system matrices when x(k+1)=Ax(k)+Bu(k)
%% Q, R, and Pf are the gains in the cost function
%% N is the length of the horizon
%% Z is the vector of optimal variables 
%% F1, G1, h1, F2, G2, h2 are constraint matrices
%% x0 is the initial condition
%% x_ub is the upper bound for absolute value of x elements
%% u_ub is the upper bound for absolute value of u elements
%% exitflag shows if the quadprog have a solution or not; it is one of quadprog outputs

n = size(A,2);
m = size(B,2);

% equality constraints
F1=[0 0];G1=0;h1=0;

  %%%%%%%% equality constraints F4
for i = 1:N

  if i == 1
      F4(i:i*n) = 0;
  elseif i == 2
      F4(i,n*(i-1)-(n-1):n*(i-1)) = F1;
  else
      F4 = blkdiag(F4,F1);
  end
  
  if i == N
      F4 = blkdiag(F4,F1);
  end
  
  %%%%%%% equality constraint G4
  if i == 1
      G4 = G1;
  else
      G4 = blkdiag(G4,G1);
  end
  if i == N
      G4(i+1,:) = 0;
  end
end

Aeq1 = blkdiag(F4,G4);

[nF4,~] = size(Aeq1); 
h4(1:nF4) = h1;
h4(1) = h4(1)-F1*x0;
h4 = h4';

%initialize
F3 = zeros(N*n,N*n);
h3 = zeros(N*n,1);

h3(1:n,1) = A*x0;
myeye = eye(n);
for i = 1:N
   %%%%%%%% system dynamics
    if i == 1
      F3(i:i*n,i:i*n) = eye(n); 
      G3 = -B;
    else
    F3(n*i-(n-1):n*i,n*(i-1)-(n-1):n*i) = [-A myeye];
     G3 = blkdiag(G3,-B);
    end
end

F3 = [F3; zeros(n,size(F3,2)-n), eye(n)];
G3 = [G3 ; zeros(n,size(G3,2))];
h3 = [h3;zeros(n,1)];

Aeq = [F3 G3;Aeq1];
beq = [h3;h4];

% inequality constraints
F2 = eye(n*N);
G2 = eye(m*N);
h2(1:n*N) = x_ub;
h2(n*N+1:(n+m)*N) = u_ub;

Aineq = blkdiag(F2,G2);
Aineq = [Aineq;-Aineq]
bineq = [h2';h2']

if N>1
H = Q;
    for i = n:2*N
       if i<N %to find Q_
           H = blkdiag(H,Q);
       elseif i == N
           H = blkdiag(H,Pf);
       else
           H = blkdiag(H,R);
       end
    end
else
    H = blkdiag(Pf,R);
end

f = zeros(size(H,1),1);
[Z,~,exitflag] = quadprog(H,f,Aineq,bineq,Aeq,beq);
end



