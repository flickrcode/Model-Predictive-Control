function [K0,P0]=DP_43(A,B,N3,Q,R,Pf)
%% do not change the inputs and outputs!
%% A and B are the system matrices when x(k+1)=Ax(k)+Bu(k)
%% Q, R, and Pf are the gains in the cost function
%% N is the length of the horizon,
%% K0 is the controller gain when u(0)=K0x(0)
%% P0 describes the final cost as VN=(x0^T)*P0*x0 
P_index_k=Pf;
for k=N3:-1:1
    K_index_k=-inv(R+B'*P_index_k*B)*B'*P_index_k*A;
    P_index_k_min_1=Q+A'*P_index_k*A-A'*P_index_k*B*inv(R+B'*P_index_k*B)*B'*P_index_k*A;
    P_index_k=P_index_k_min_1;
end
K0=K_index_k;
P0=P_index_k;
end