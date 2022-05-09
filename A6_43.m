clear;clc;close all;
%% Q1
% define H and V that are correspondent to the polyhedron for H and v representation. Plot the polyhedron and
% explain the difference in the report.

A = [0 1;-1 0;-1 -1;1 1];
b = [0;0;1;1];

% Find H-representation
P = Polyhedron(A,b);
H = P;

% Find V-representation
V = Polyhedron(P.V);

figure(1);
plot(H)
xlabel('x_1');
ylabel('x_2');
title('H-Representation of Polyhedron');

figure(2);
plot(V)
xlabel('x_1');
ylabel('x_2');
title('V-Representation of Polyhedron');

%% Q2
% define Q and P, find the sum and the difference and plot the results.
% Sketch the plots in the report as well
A1 = [0 1;1 0;0 -1;-1 0];
b1 = [2;2;2;2];
P = Polyhedron(A1,b1);

A2 = [-1 -1;1 1;1 -1;-1 1];
b2 = [1;1;1;1];
Q = Polyhedron(A2,b2);


% Minkowiski sum 
Minkowski_sum = plus(P,Q);

% Pontryagin difference 
Pontraygin_diff = minus(P,Q);

figure(3);
plot(Minkowski_sum,'color','b',Pontraygin_diff)
legend('Minkowiski sum','Pontryagin difference','Location','northwest');
xlabel('x_1');
ylabel('x_2');
title('Minkowiski sum & Pontryagin difference');

%% Q3
% write a code that shows S is invariant and explain your approach in the
% report
A3 = [0.8 0.4 ; -0.4 0.8];
Aineq3 = [eye(2);-eye(2);1 1;1 -1;-1 1;-1 -1];
bineq3 = [1;1;1;1;1.5;1.5;1.5;1.5];

% Non-autonomous system x^(+)=Ax
model = LTISystem('A',A3)

% Polyhedron of Set S
S = Polyhedron(Aineq3,bineq3);


%%%%% Alternative way of doing %%%%
figure(3)
Splus = Polyhedron(Aineq3*A3^-1,bineq3);
plot(S,'color','b',Splus,'color','y');
legend('S','S_{plus}');
xlabel('x_1');
ylabel('x_2');
title('Finding Invariant Set S');
check1 = S.contains(Splus);


%% Q4
% Fill in the Reach_X function and Plot S and its one step reachable set.
% Note that you are not supposed to change the inputs and outputs of the
% function.

A4 = [0.8 0.4 ; -0.4 0.8];
B4 = [0;1];
Ain4 = [eye(2);-eye(2);1 1;1 -1;-1 1;-1 -1];
bin4 = [1;1;1;1;1.5;1.5;1.5;1.5];

%Find a V-representation of input inequality constraint set U
UU = Polyhedron('lb',-1,'ub',1);
U = Polyhedron(UU.V)

% Find V-representation of state inequality constraint Set S
S = Polyhedron(Ain4,bin4);
V = Polyhedron(S.V)

Reach4 = Reach_43(A4,B4,V,U);

figure (4)
plot(S,'color','r',Reach4,'color','y','alpha',0.5);
legend('S','Reach of S');
xlabel('x_1');
ylabel('x_2');
title('S and Reach of S');

%% Q5
% Fill in the Pre_X function and Plot S and its Pre set.
% Note that you are not supposed to change the inputs and outputs of the
% function.

% Call pre-defined function Pre
Pre5 = Pre_43(A4,B4,S,UU);


figure(5)
plot(Pre5,'color','lightblue',S,'color','y');
legend('Pre of S','S');
xlabel('x_1');
ylabel('x_2');
title('S and Pre of S');


%% Q6
A6 = [0.9 0.4;-0.4 0.9];
B6 = [0;1];
n = size(A6,2);
Pf = zeros(n,n);
Q = eye(n);
R = 1;

Xf = Polyhedron([eye(n);-eye(n)],zeros(2*n,1));
x0 = [2;0];
x_ub = 3;
u_ub = 0.1;

% part 1: Fill in the function ShorterstN_XX.m and use it to find the shortest N
% that is feasible. Note that you are not supposed to change the inputs and outputs of the
% function.

for N1=3:-1:1
    [Z1,Aineq,bineq,exitflag1]=ShortestN_43(A6,B6,N1,Q,R,Pf,x_ub,u_ub,x0)
    x(:,N1+1) = Z1(1:n);
    u_vector(N1) = Z1(N1*n+1);
    
    if exitflag1 ~= 1
        N1 = N1+1;
        break
    end 
end
%%
% part 2: Fill in the function RHCXf_X.m and use it to check the
% feasibility. Note that you are not supposed to change the inputs and outputs of the
% function.

N2=2;
N=N2;
A=A6;
B=B6;
[Z2,exitflag2]=RHCXf_43(A6,B6,N2,Q,R,Pf,x_ub,u_ub,Xf,x0);

%%
% part 3: Plot the feasible sets for the initial condition in part 1 and 2
% and plot those sets. Answer to the rest of the question in the report.

% Find the initial feasible states for the first part (4.1)
% x_init = [1.8 1.8;-0.6995 -0.7000]
% 
% A7_final = 
% Polyhedron_1 = Poly
%%% create model in MPT3 interface
model = LTISystem('A',A6,'B',B6);
% 
Aineq6 = Aineq(1:6,1:3)
bineq6 = bineq(1:6)

% Find H-representation of unified inequality constraint Set S
S6 = Polyhedron(Aineq6,bineq6);

% Modify Pre(x)
%  P=Pre_43(A6,B6,S6,U6)
% H = S6.H
% U = U6.

% backward reachable set (preset)
% Xpre = model.reachableSet('X', S6, 'direction', 'backward','N',1);
% 
% PH = [H.*A6 H.*B6 ;zeros(size(H,1),size(Hs*A6,2)) U];
% Ph = [1;1];
% ProjPoly = Polyhedron(PH,Ph);
% 
% % P = projection(ProjPoly,[1;2]);
% 
% figure(6)
% plot(Xpre,'color','r')
% xlabel('x_1');
% ylabel('x_2');
% title('H-Representation of Xpre for 1st RHC')

%%% Find the initial states for the second part (4.2)



 
