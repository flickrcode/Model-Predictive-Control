function [K0,P0]=DP_43(A,B,N,Q,R,Pf)
%% do not change the inputs and outputs!
%% A and B are the system matrices when x(k+1)=Ax(k)+Bu(k)
%% Q, R, and Pf are the gains in the cost function
%% N is the length of the horizon,
%% K0 is the controller gain when u(0)=K0x(0)
%% P0 describes the final cost as VN=(x0^T)*P0*x0 

P_index_k=Pf;
for i=N:-1:1
    Pks1=Q+A'*P_index_k*A-A'*P_index_k*B*inv(R+B'*P_index_k*B)*B'*P_index_k*A;
    P_index_k=Pks1;
    
    P_index_k_plus_1=P_index_k;
    K_index_k=-inv(R+B'*P_index_k_plus_1*B)*B'*P_index_k_plus_1*A;
end
P0=P_index_k;
K0=K_index_k;

