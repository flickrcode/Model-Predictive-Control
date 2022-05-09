function [K0,P0]=BS_43(A,B,N,Q,R,Pf)
%% do not chane the inputs and outputs!
%% A and B are the system matrices when x(k+1)=Ax(k)+Bu(k)
%% Q, R, and Pf are the gains in the cost function
%% N is the length of the horizon
%% K0 is the controller gain when u(0)=K0x
%% P0 describes the final cost as VN=(x0^T)*P0*x0 
% Define that the first element of Omega is equal to A
A=[1.0025 0.1001;0.05 1.0025];
B=[0.005;0.1001];
Omega=A;

% Use the for loop to construct the Omega matrix
for i=2:N
    X=[kron(A^N,1)];
    Omega=[Omega;X];
    
end

% Define the initial Gamma sizes
sizeB=size(B);
Gamma=zeros(length(A)*N,sizeB(2)*N);
Gamma=Gamma+kron(eye(N),B);

% Use the for loop to construct the Gamma matrix
for i=1:N-1
    Gamma=Gamma+kron(diag(ones(N-i,1),-i),A^i*B);
end

% Define the weightings for the LQ criterion
Qbar=blkdiag(kron(eye(N-1),Q),Pf);
Rbar=kron(eye(N),R);

% Find the desire value of K0 and P0
K0 = -inv(Gamma'*Qbar*Gamma+Rbar)*(Gamma'*Qbar*Omega);
P0 = Q + Omega'*Qbar*Omega - Omega'*Qbar*Gamma*inv(Gamma'*Qbar*Gamma + Rbar)*Gamma'*Qbar*Omega;


end
