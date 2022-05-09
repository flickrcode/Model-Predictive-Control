clear all;
clc;
%% Note that when you run this file, N2, N3, Pf3, and N5 should be defined in Workspace and have the correct values.
%% Q1: Fill the DP_X.m function using the dynamic programic approach.


%% Q2: Find the shortest N that stabilizes the system using DP_X.m ; define N2 that gives the shortest N, i.e. N2=min(N) subject to the system stability
A=[1.0025 0.1001;0.05 1.0025];
B=[0.005;0.1001];
C=[1 0];
D=0;
Q=[5 0;0 1];
Pf=[5 0;0 1];
R=0.5;

% Call the defined function in DP_43.m and test for different values of N
N2= 3;
[K0,P0]=DP_43(A,B,N2,Q,R,Pf)

% Check for eigenvalues
system_eig_2 = eig(A+B*K0)

%% Q3: Define Pf3 as the solution to the Riccati Equation; define N3 that gives the shortest N, i.e. N3=min(N) subject to the system stability
% Use dare function to find X and test for different length of N
[X,L,G]=dare(A,B,Q,R);

%Test length of N
N3=1;

%Define Pf as the terminal cost function
Pf3=X;

%Call the defined function
[K0,P0]=DP_43(A,B,N3,Q,R,Pf)

%Check for Eigenvalues
system_eig_3 = eig(A+B*K0)

%% Q4: Fill the BS_43.m function using the batch solution approach.
% Look at BS_43.m

%% Q5: Find the shortest N that stabilizes the system using BS_XX.m; define N5 that gives the shortest N, i.e. N5=min(N) subject to the system stability
% Test for length of N
N5=4;

% Call defined function
[K0,P0]=BS_43_part1(A,B,N5,Q,R,Pf);

% Check for eigenvalues
system_eig_2 = eig(A+B*K0(1,:))


%% Q6: Use BS_XX.m o DP_XX.m and simulate the system for 20 steps; plot the inputs and the states for these four cases.
x0=[1;0];

% Plot for each tuning of RHC controllers from the defined function
    hold on
    R=0.5;N=5;
    [K0,P0]=BS_43_part2(A,B,C,N,Q,R,Pf,x0);
    R=0.5;N=15;
    system_eigg_1 = eig(A+B*K0(1,:))
    
    [K0,P0]=BS_43_part2(A,B,C,N,Q,R,Pf,x0);
    R=0.05;N=5;
    system_eigg_2 = eig(A+B*K0(1,:))
    
    [K0,P0]=BS_43_part2(A,B,C,N,Q,R,Pf,x0);
    R=0.05;N=15;
    system_eigg_3 = eig(A+B*K0(1,:))
    
    [K0,P0]=BS_43_part2(A,B,C,N,Q,R,Pf,x0);
    system_eigg_4 = eig(A+B*K0(1,:))
    
    legend('R=0.5,N=5','R=0.5,N=15','R=0.05,N=5','R=0.05,N=15','Location','southwest','FontSize',12);
    xlabel('Time in [seconds]');
    ylabel('Output [-]')
    title('Plot of system input-output for Receding Horizon Controller')
    hold off

%% Q7: Fill the CRHC1_X.m function
N = 2;
[Z,VN]=CRHC1_43(A,B,N,Q,R,Pf,F1,G1,h1,F2,G2,h2,x0)

%% Q8: Fill the CRHC2_X.m function
%% Q9: Solve Q6 using CRHC1_X.m or CRHC2_X.m considering the given constraints for 100 sample times
tf=100;
x0=[1;0];





