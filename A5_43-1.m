clc;clear;close all;
%% Q1
% you are supposed to find Matrix S such that V(x(k)) is a Lyapunov
% function that V is zzdecreasing (except at the origin) in the report;
%In addition, define matrix S here as well.
A = [0.5 1;-0.1 0.2]
Q= [1 0;0 1]

S = dlyap(A,Q);


%% Q2
% answer in the report;
Q =[1 0;0 1];
A=[1 1;-1 5];
B=[0;1];
R=1;

[P,~,~,~] = idare(A,B,Q,R,[],[])

%% Q3 
% part a:
% Define N (the shortest horizon that ...) here as N3. You can use DP_XX.m
% that you have writen in previous assignments. Do note that when I run
% your code, I should be able to see the result and just writing a number
% would not be enough. Mention this N in your report as well.
A=[1 1;-1 5];
B = [0;1];
x_1=1
Q = x_1*eye(size(A,1));
R = 1;
Pf = Q;

N3=5;
[K0,P0]=DP_43(A,B,N3,Q,R,Pf)
%
system_eig_1 = eig(A+B*K0)
modulus_eig_1 = abs(system_eig_1)
% 
% 
% % part b:
% % explain in the report
% 
% % part c:
% % Fill in the function Pf_X.m in which X is your group number. Motivate
% % the concept behind your code in the report.
Pf_2=Pf_43(A,B,Q,R)
% 
% % part d:
% % you can use trial and error to find this R; just provide the R that works
% % and check the stability by checking the eigenvalues of the closed loop
% % system with this new R; define it in the code as Rnew
N3new=1;
Rnew=0.19;
[K0_2,P0_2]=DP_43(A,B,N3new,Q,Rnew,Pf);

system_eig_2 = eig(A+B*K0_2);
modulus_eig_2 = abs(system_eig_2)
% 
% %% Q4 
% % write your proof in the report
% 
% %% Q5
% % answer to the question in the report. Do note that you can verify your
% % answer by checking it numerically in Matlab (this is just for your own
% % and you may not provide any code regarding this)

Rtest = linspace(0.01,0.5,100);
Pf5 = Q;
for i=1:length(Rtest)
    K5a(i,:) = -(Rtest(i) + B'*Pf5*B)^-1*B'*Pf5*A;
    eigens5 = abs(eig(A+B*K5a(i,:)));
    if eigens5(1)>1 || eigens5(2)>1
        Rrange = [0 Rtest(1:i-1)];
        break
    end
end
% so R range is [0;0.2]

for i=1:length(Rtest)
    K5a(i,:) = -(Rtest(i) + B'*Pf5*B)^-1*B'*Pf5*A;
    eigens5 = abs(eig(A+B*K5a(i,:)));
    if eigens5(1) <= 1 & eigens5(2) <= 1
        plotX(i,:) =  eigens5;
    else
        plotX(i,:) =  [0 0];
    end
end
figure (1);
 plot(Rtest,plotX)
 legend('Pole 1','Pole 2')
 xlabel('R test values')
 ylabel('Eigenvalues')
 title('Test Check for Eigen Values < 1 with N=1')
N5b = 2;
Rtestb = linspace(0.01,5,500);
Pf5b = Q;
for i=1:length(Rtestb)
    [K5b(i,:),P05b]=DP_43(A,B,N5b,Q,Rtestb(i),Pf5b);
    eigens5b = abs(eig(A+B*K5b(i,:)));
    if eigens5b(1) <= 1 & eigens5b(2) <= 1
        plotXb(i,:) =  eigens5b;
    else
        plotXb(i,:) =  [0 0];
    end
end
figure (2);
 plot(Rtestb,plotXb)
  legend('Pole 1','Pole 2','Location','best')
 xlabel('R test values')
 ylabel('Eigenvalues')
 title('Test Check for Eigen Values < 1 with N=2')

% so the range is between [0,0.38]U[2.61,4.49]
 

% %% Q6
% % answer in the report
% 
% 
% 
% %% do not comment this! when you run this script, the printed values should be the correct answers!
S
N3
Rnew

% 
% 
