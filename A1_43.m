clear;
clc;
close all;
%%You are supposed to fill this template. Remeber not to change variable
%%names! When you run this file, you should see all these variables in the Workspace with
%%the right values! 
%% Discrete Model
Ac=[0 1;0.5 0];Bc=[0;1]; Cc=[1 0];
Dc=0;
h=0.1;

%Use given parameters and find A, B, and C in Question #1
sys=ss(Ac,Bc,Cc,Dc);
sysd=c2d(sys,h);
 A= sysd.A 
 B= sysd.B
 C= sysd.C
 

%% Delayed model
% define Aa, Ba, Ca, Da according to Question #2
syms s
% % 
Ts=0.5*h;
A_a = expm(Ac*h) 
Ba_a = double(expm(Ac*(h-Ts)))
Ba_b = double(int(expm(Ac*s)*Bc,0,Ts))

Ba_1 = Ba_a*Ba_b
Ba_2 = double(int(expm(Ac*s)*Bc,0,h-Ts))

Aa = [A_a Ba_1;0 0 0]
Ba = [Ba_2;eye]
Ca = [Cc 0];

% tau = 0.5*h;
% ninputs = 1;
% nstates = 2;
% noutputs = size(Cc,1);
% 
% [Ad,dummy] = c2d(Ac,Bc,h); 
% [A1,B1] = c2d(Ac,Bc,tau);
% [A2,B2] = c2d(Ac,Bc,h-tau); 
% 
% B1 = A2*B1; 
% Aa = [Ad, B1; zeros(ninputs,nstates+ninputs)];
% Ba = [B2; eye(ninputs)];
% Ca = [Cc,zeros(noutputs,ninputs)];


% %% Controllability and Observability
% %find the rank of controllablity and observability matrices according to
% %Question #3
% 
% 
% % the following parameters should be the ranks of the observability and 
% % the controllability matrices, accordingly
% 
sys2_c= ctrb(Ac,Bc)
sys2_o= obsv(Ac,Cc)
sys3_c= ctrb(A,B);
sys3_o= obsv(A,C);
sys4_c= ctrb(Aa,Ba);
sys4_o= obsv(Aa,Ca);

if rank(sys2_c)== length(Ac)
    controlability_sys2_c =1 
    disp('system is controllable')
else controlability_sys2_c = 0
    disp('system is not controllable')
end

if rank(sys3_c)== length(A)
    controlability_sys3_c = 1
     disp('system is controllable')
else controlability_sys3_c = 0
    disp('system is not controllable')
end

if rank(sys4_c)== length(Aa)
    controlability_sys4_c = 1
     disp('system is controllable')
else controlability_sys4_c = 0
    disp('system is not controllable')
end

if rank(sys2_o)== length(Ac)
    Observability_sys2_o =1 
    disp('system is observable')
else Observability_sys2_o = 0
    disp('system is not observable')
end

if rank(sys3_o)== length(A)
   Observability_sys3_o = 1 
    disp('system is observable')
else Observability_sys3_o = 0
    disp('system is not observable')
end

if rank(sys4_o)== length(Aa)
   Observability_sys4_o = 1 
    disp('system is observable')
else Observability_sys4_o = 0
    disp('system is not observable')
end

% Define Cc such that the continous time system become unobservable
% We can use try to find big condition number
Ac_test=[0 1;0.5 0];
Cc_test = [1 0];

obsv_test = obsv(Ac_test,Cc_test)
obsv_test_rank = rank(obsv_test)

% Test for system 3
C_3 = [1 sqrt(2)];
obsv_test3 = obsv(A,C_3)

if rank(obsv_test3)== length(A)
   Observability_test_3 = 1 
    disp('system is observable')
else Observability_test_3 = 0
    disp('system is not observable')
end

% Test for system 4
C_4 = [C_3 0];
obsv_test4 = obsv(Aa,C_4)

if rank(obsv_test4)== length(Aa)
   Observability_test_4 = 1 
    disp('system is observable')
else Observability_test_4 = 0
    disp('system is not observable')
end

% % Question 5
BB = C_3*B
AA = C_3*A
% 
% %% Controller design
landa1=-4+6*1i;
landa2=-4-6*1i;
% 
% %calculate desired poles for the discrete time system (3) and define them as p1 and
% %p2 as it is asked in Question #6
p = [landa1 landa2];
% 
% % Convert continuous poles to Discrete time poles
p1 = exp(p(1)*h); 
p2 = exp(p(2)*h);
P = [p1 p2]

% %define the feedback gain for the discrete time system (3) as K1
K1 = place(A,B,P)
% 
% %define the feedback gain for the delayed discrete time system (4) as K2
K2 = [K1 0];
% 
% % Define the reference gain for system (3)
E = eig(A)

D = 0;
Acl_3 = A-(B*K1);
syscl_3 = ss(Acl_3,B,C,D);

Ecl = eig(Acl_3);

Kdc_3 = dcgain(syscl_3);
Kr_3 = 1/Kdc_3;

sys_cl_3 = ss(Acl_3,B*Kr_3,C,D,h)
transf_3 = tf(sys_cl_3)
poles_3 = pole(transf_3)

% % Define the reference gain for system (4)
Da =0;
Acl_4 = Aa-(Ba*K2);
syscl_4 = ss(Acl_4,Ba,Ca,Da);

Kdc_4 = dcgain(syscl_4);
Kr_4 = 1/-Kdc_4;

sys_cl_4 = ss(Acl_4,Ba*Kr_4,Ca,Da,h)
transf_4= tf(sys_cl_4)
poles_4 = pole(transf_4)

% % Define the new state feedback gain for system (4) 

P_new = [p1 p2 0]
K3 = place(Aa,Ba,P_new)

Acl_5 = Aa-(Ba*K3);
syscl_5 = ss(Acl_5,Ba,Ca,Da);

sys_cl_5 = ss(Acl_5,Ba*Kr_4,Ca,Da,h)
transf_5= tf(sys_cl_5)

% %plot the step response of the systems in one figure. Your figure should
% %have labels and legend.
figure(1)
step(sys_cl_3)
hold on
step(sys_cl_4)
step(sys_cl_5)
legend('Discrete time model','Discrete time model incl. delay',...
    'Discrete time model incl. delay with new feedback gain','Location','northeast')

% %% Steady State
ys=pi/6;
% % plot the system output as explained in Question #7. Your figure should
% %have labels and legend.
steady_state = [(eye(3)-Aa) -Ba; Ca 0]
output = [0;0;0;ys]

state_input = inv(steady_state)*output
xs = [state_input(1);state_input(2);state_input(3)]
us = state_input(4)

% Use for loop to plot the output of steady state

n=100;
t=linspace(0,n-1,n-1);
xa=zeros(3,n);
ya=zeros(3,n);

for i=1:n-1
    xa(:,i+1)=(Aa-Ba*K3)*(xa(:,i)-xs)+xs;
    ya(:,i)=Ca*(xa(:,i)-xs)+ys;
end     

figure (2)
plot(t,ya(:,1:n-1))
title('Steady State Behaviour of System (4)')
xlabel('seconds')
ylabel('radian');


% %% disturbance
Bd=[0;1;0]; Cd = 0
% % define Ae, Be, Ce, and De as asked in Question #8
Ae= [Aa Bd;0 1 1 1]
Be= [Ba;0]
Ce= [Ca Cd]
De= 0

% Since it is controllable, hence the system is stabilizable
lambda_all = eig(Ae)    
lambda_non_negative = [1.0025 2.0038 0]
 
% Hautus test
sys6_c=rank(ctrb(Ae,Be));
sys6_o=rank(obsv(Ae,Ce));

if sys6_c == length(Ae)
    controlability_sys6_c = 1
     disp('system is controllable')
else controlability_sys6_c = 0
    disp('system is not controllable')
end

if sys6_o== length(Ae)
   Observability_sys6_o = 1 
    disp('system is observable')
else Observability_sys6_o = 0
    disp('system is not observable')
end

% Hautus Test for Stabilizability and Detectability

 Stabilizability_test_1 = [lambda_non_negative(1).*eye(4)-Ae Be] 
 Stabilizability_test_2 = [lambda_non_negative(2).*eye(4)-Ae Be] 
 Stabilizability_test_3 = [lambda_non_negative(3).*eye(4)-Ae Be] 

 Detectability_test_1 = [lambda_non_negative(1).*eye(4)-Ae;Ce] 
 Detectability_test_2 = [lambda_non_negative(2).*eye(4)-Ae;Ce] 
 Detectability_test_3 = [lambda_non_negative(3).*eye(4)-Ae;Ce] 
 
  if rank(Stabilizability_test_1) == length(Ae)
   Stabilizability_1 = 1 
    disp('system is Stabilizable')
else Stabilizability_1 = 0
    disp('system is not Stabilizable')
 end
  if rank(Stabilizability_test_2) == length(Ae)
   Stabilizability_2 = 1 
    disp('system is Stabilizable')
else Stabilizability_2 = 0
    disp('system is not Stabilizable')
  end
  if rank(Stabilizability_test_3) == length(Ae)
   Stabilizability_3 = 1 
    disp('system is Stabilizable')
else Stabilizability_3 = 0
    disp('system is not Stabilizable')
 end
 if rank(Detectability_test_1) == length(Ae)
   Detectability_1 = 1 
    disp('system is detectable')
else Detectability_1 = 0
    disp('system is not detectable')
 end
  if rank(Detectability_test_2) == length(Ae)
   Detectability_2 = 1 
    disp('system is detectable')
else Detectability_2 = 0
    disp('system is not detectable')
  end
  if rank(Detectability_test_3) == length(Ae)
   Detectability_3 = 1 
    disp('system is detectable')
else Detectability_3 = 0
    disp('system is not detectable')
 end
% 
% %% Feedback gain
% %define the controller gain as K3 according to Question #9
K4=place(Ae,Be,[P_new 1])

Acl_6 = Ae-(Be*K4);
syscl_6 = ss(Acl_6,Be,Ce,De);

Kdc_6 = dcgain(syscl_6);
Kr_6 = 1/Kdc_6;

sys_cl_6 = ss(Acl_6,Be*Kr_6,Ce,De,h)

figure(3)
step(sys_cl_6)


%% Observer
%define L as the observer gain according to Question #10

L=place(Ae.',Ce.',[0.1,0.2,0.3,0.4])'

eig_obs = eig(Ae-L*Ce)
