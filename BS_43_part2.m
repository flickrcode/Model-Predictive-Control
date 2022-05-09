function [K0,P0]=BS_43_part2(A,B,C,N,Q,R,Pf,x0)

% Define that the first element of Omega is equal to A
A=[1.0025 0.1001;0.05 1.0025];
B=[0.005;0.1001];
O=A;

% Use the for loop to construct the Omega matrix
for i=2:1:N
    X=[kron(A^i,1)];
    O=[O;X];
end

% Define the initial Gamma sizes
sizeB=size(B);
G=zeros(length(A)*N,sizeB(2)*N);
G=G+kron(eye(N),B);

% Use the for loop to construct the Gamma matrix
for i=1:N-1
    G=G+kron(diag(ones(N-i,1),-i),A^i*B);
end

% Define the weightings for the LQ criterion
Qbar=blkdiag(kron(eye(N-1),Q),Pf);
Rbar=kron(eye(N),R);

% Find the desire value of K0 and P0
K0=-inv(G'*Qbar*G+Rbar)*G'*Qbar*O;
P0=(Q+O'*Qbar*O-O'*Qbar*G*inv(G'*Qbar*G+Rbar)*G'*Qbar*O);

% Define the initial condition and the first control action in the optimal
% control sequence
x=[1;0];
u=-inv(G'*Qbar*G+Rbar)*G'*Qbar*O*x(:,1);

% Use for loop to solve for the state x and input u
for i=1:N
    xp=A*x(:,i)+B*u(i,:);
    x=[x,xp(:,i)];
    u=[u,K0*x(:,i+1)]
end
 
% Define the Output equation using C matrix
y=C*x;

% Plot the simulation of input-output of the system for 20 seconds
t=linspace(0,20,N+1);

plot(t,y)
end
