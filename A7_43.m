close all;
clear all;
clc;
%% You can use any command in MPT in this assignment
%% Question 1
% write the code to plot X0. In the report, give Pf,  the X0, and your
% motivation for choosing this Pf.

% Define the parameters 
A = [1.2 1;0 1]; n = size(A,2);
B = [0;1]; m = size(B,2);
Q = eye(n);
R = 100;
N1 = 3;

% Solve the Discrete Riccati equation
Pf1 = idare(A,B,Q,R)

% Define the Terminal constraint, input constraint and state constraint as
% polyhedron
Xf1 = Polyhedron([eye(n);-eye(n)],zeros(2*n,1));
U = Polyhedron('lb',-1,'ub',1);
S = Polyhedron('lb',[-15 -15],'ub',[15 15]);

% Use the Pre(x) function from the Assignment 6
P1 = Pre_43(A,B,Xf1,U);
P2 = Pre_43(A,B,P1,U);
P3 = Pre_43(A,B,P2,U);

% Plot the Pre value for different constraints
figure(1)
plot(P3,'color','lightblue',P2,'color','y',P1,'color','g',Xf1,'color','r')
xlabel('x_1');
ylabel('x_2');
title('Finding X_0 starting from X_f')
legend('X_{f} (3) = X_{0}', 'X_{f} (2)','location','northeast')


%% Question 2
% write the code to plot the requested figures. Provide the figures in your
% report and explain your observation about the error between the state and
% state prediction as N increases.

% Define the LTI model and its upper/lower bound constraints
x0 = [4; -2.6];
model = LTISystem('A', A, 'B', B);
model.x.min = [-15; -15];
model.x.max = [15; 15];
model.u.min = -1;
model.u.max = 1;

% Define the penalty Q and R using QuadFunction
model.x.penalty = QuadFunction(Q);
model.u.penalty = QuadFunction(R);



% Design the discrete LQ controller, with its output gain K and its Riccati
% solution P. 
[K,P] = dlqr(A,B,Q,R);

% Include the result of the gain as terminal penalty and set in the model
% as well as use the Xf1 as terminal set %%%%%
model.x.with('terminalPenalty');
model.x.terminalPenalty = QuadFunction(P);
model.x.with('terminalSet');
model.x.terminalSet = Xf1;



% Design MPC controller for each different Prediction Horizon Length%%%
%%%%%%%% using ready-made 'MPCController' function %%%%%%%%%%%%%%%%

%%%%% For  N = 10 %%%%%
N_1 = 10;
pred1 = MPCController(model, N_1);
[u1, feasible, openloop_1] = pred1.evaluate(x0);

% Create a Closed Loop system using the ready made 'ClosedLoop'
loop1 = ClosedLoop(pred1, model);
closedloop_1 = loop1.simulate(x0, N_1);

figure (2)
plot([0:N_1],openloop_1.X,[0:N_1],closedloop_1.X)
ylabel('States')
legend('x_{1,p}','x_{2,p}','x_{1,MPC}','x_{2,MPC}','location','northwest')
title('Predictions vs Actual States for N=10')
grid on

%
%%%%% For N = 15 %%%%%
N_2 = 15;
pred2 = MPCController(model, N_2);
[u2, feasible, openloop_2] = pred2.evaluate(x0);

loop2 = ClosedLoop(pred2, model);
closedloop_2 = loop2.simulate(x0, N_2);

figure (3)
plot([0:N_2],openloop_2.X,[0:N_2],closedloop_2.X)
ylabel('States')
legend('x_{1,p}','x_{2,p}','x_{1,MPC}','x_{2,MPC}','location','northwest')
title('Predictions vs Actual States for N=15')
grid on

%%%%% For N = 20 %%%%%
N_3 = 20;
pred3 = MPCController(model, N_3);
[u3, feasible, openloop_3] = pred3.evaluate(x0);

loop3 = ClosedLoop(pred3, model);
closedloop_3 = loop3.simulate(x0, N_3);

figure (4)
plot([0:N_3],openloop_3.X,[0:N_3],closedloop_3.X)
ylabel('States')
xlabel('Time')
legend('x_{1,p}','x_{2,p}','x_{1,MPC}','x_{2,MPC}','location','northwest')
title('Predictions vs Actual States for N=20')
grid on


%% Question 3
% no code is needed. Answer in the report
%% Question 4
% write a code that calculates the figures and costs. Provide the figures
% and costs in the report. for costs, provide a table in the report that
% provides all costs for all different methods in the question (4 methods,
% each with three different costs as defined in A7 assignment). If you what
% to use some functions in the code, you can write them in different matlab
% files and submit them with the rest of your files

%% PART 1: 3 MPC controllers %%%

clear all;
clc;

%%%  defining the MPC design parameters  %%%
load('A7_data.mat');
dt = 1; %hour
R = 50;
C = 9.2e3;
T_ub = 26; % Upper boundary of the temperature %
T_lb = 21; % Lower boundary of the temperature
N = 24; % Prediction horizon

%%% Define the System Model %%%%
A = 1 - dt*3600./(R*C);
B = dt*3600./C;
d = Pd*dt*3600./C+T_oa*dt*3600./(R*C);

%%%% Closed-Loop Simulation Parameters %%%%%

kappa = 2;
rho = 1000;
T0 = 22; %
steps = 24*3;

%%% Define the template size of temperature,inputs and distrubance as zeros %%%
%%%%%%%%%%%%%%%%%%% and the initial value as T0 %%%%%%%%%%%%

Temps1 = zeros(1, steps+1);
Temps2 = zeros(1, steps+1);
Temps3 = zeros(1, steps+1);

% Initial temperatures
Temps1(1) = T0;
Temps2(1) = T0;
Temps3(1) = T0;

% Inputs template size
inputs1 = zeros(1, steps);
inputs2 = zeros(1, steps);
inputs3 = zeros(1, steps);

% Disturbances template size
current_disturbance_1 = zeros(24,1);
current_disturbance_2 = zeros(24,1);
current_disturbance_3 = zeros(24,1);

%%%  Start the Control loop  %%%
for i=1:steps
    
    %%%%%%%%%%%%%  After a transient period %%%%%%%%%%%%%
    %%%%%%%%%%% system will settle at steady state %%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%% Sampling Time = 1 hour %%%%%%%%%%%%
    
    check = floor(i/24);
    if rem(i,24) == 0
        i_day = 1;
    else
        i_day = i - (24*check)+1;
    end
    
    % Collect the array of disturbances
    if i == 1
        current_disturbance_1 = d;
    else
        current_disturbance_1 = [d(i_day:end) ; d(1:i_day-1)];
    end
    
    %%% CASE 1 (Predictions of Toa and Pd are perfectly known, no mismatch) %%%%%
    [inputs1(i),Temps1(i+1)] = CRHC7_43(A,B,current_disturbance_1,N,kappa,rho,T_ub,T_lb,Temps1(i));
    
    %%% CASE 2 (Only current values are known) %%%
    current_disturbance_2 = current_disturbance_1(1)*ones(24,1);
    [inputs2(i),Temps2(i+1)] = CRHC7_43(A,B,current_disturbance_2,N,kappa,rho,T_ub,T_lb,Temps2(i));

    %%% CASE 3 (Toa and Pd are unknown) %%%
    [inputs3(i),~] = CRHC7_43(A,B,current_disturbance_3,N,kappa,rho,T_ub,T_lb,Temps3(i));

    Temps3(i+1) = A*Temps3(i) + B*inputs3(i) + current_disturbance_1(1);
    
    %%%%%%%% d(k) = T(k+1)-AT(k)-Bu(k)%%%%%%%%%%%
    current_disturbance_3 = (Temps3(i+1)-A*Temps3(i)-B*inputs3(i))*ones(24,1);
end

%%%  plotting the results of the Control Loop %%%
figure (5)
plot([0:steps],Temps1,[0:steps],Temps2,[0:steps],Temps3)
title('Temperature Inside the Room');
xlabel('Time [hrs]');
xlim([0 70])
ylabel('Temperature [°C]');
legend('Case 1','Case 2','Case 3','Location','south');
grid on

figure (6)
plot([0:steps-1],inputs1,[0:steps-1],inputs2,[0:steps-1],inputs3)
title('Control Input Power Demand');
xlabel('Time [hrs]');
ylabel('Input Magnitude [kW]');
legend('Case 1','Case 2','Case 3','Location','southwest');
grid on


%% PART 2: P controller %%%

%%% Initial  Parameters for P-controller %%%
dt = 1; %minute
Ac1 = 1 - dt*60./(R*C);
Bc1 = dt*60./C;
dc1 = Pd*dt*60./C+T_oa*dt*60./(R*C);
steps1 = 60*24*3; % 1 day iteration
K = 400;

%%% Initalization parameter for the loop %%%
Tc1 = zeros(1, steps1+1);
Tc1(1) = T0;
u1 = zeros(1, steps1);

%%% Start Control Loop %%%

for i=1:steps1
    
    %%%%%%%%%%%%%  After a transient period %%%%%%%%%%%%%
    %%%%%%%%%%% system will settle at steady state %%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%% Sampling Time = 1 minute %%%%%%%%%%%%
    
    check2 = rem(i,24);
     if check2 ~= 0
          i_now = floor(i/60)+1;
          check3 = rem(i_now,24);
            if i_now > 24 && check3 ~= 0
                i_now = i_now - floor(i_now/24)*24;
            end
            if check3 == 0
                i_now = 24;
            end
     else
         i_now = 24;
     end
     
    %%%%%%%%%%%% Check for Defined 3 Input Parameter Conditions %%%%%%%%%%
    %%% T(t)>T_bar %%
    if Tc1(i)>T_ub
        u1(i) = K*(T_ub - Tc1(i));
    %%% T_underbar < T(t) < T_bar    
    elseif Tc1(i)<= T_ub && Tc1(i) >= T_lb
        u1(i) = 0;
    %%%% T(t) < T_underbar %%%%%%    
    else 
        u1(i) = K*(T_lb - Tc1(i));
    end
      %%%%%%%% T(k+1) = AT(k) + Bu(k)+ d(k)%%%%%%%%%%%
    Tc1(i+1) = Ac1*Tc1(i) + Bc1*u1(i) + dc1(i_now);
end


%%% Plot the Temperature and Control input for P Controller %%%

figure(7)
plot([0:steps1],Tc1)
title('Temperature Inside the Room');
xlabel('Time [minutes]');
ylabel('Temperature [°C]'); %alt + 248
legend('P controller','Location','southeast');
grid on

figure(8)
plot([0:steps1-1],u1)
title('Control Input Power Demand');
xlabel('Time [Minutes]');
ylabel('Input Magnitude [kW]');
legend('P controller','Location','southwest');
grid on


%%  PART 3: Performance Evaluations %%

%%% Total energy consumption %%%
% Use the trapezoidal integration method to sum the inputs and calculate
% the energy consumptions from corresponding P and MPC controllers%

tot_energycons_mpc1 = trapz(abs(inputs1(48:72))) % 1st MPC
tot_energycons_mpc2 = trapz(abs(inputs2(48:72))) % 2nd MPC
tot_energycons_mpc3 = trapz(abs(inputs3(48:72))) % 3rd MPC
tot_energycons_p = trapz(abs(u1(48*60:72*60)))/60 % P-Controller


%%% Peak Power Consumption Calculation %%%

% The peak power consumption could be calculated from finding the maximum
% value from the inputs array that we have from corresponding P and 
% MPC controller %

peak_mpc1 = max(abs(inputs1(48:72))) % 1st MPC
peak_mpc2 = max(abs(inputs2(48:72))) % 2nd MPC
peak_mpc3 = max(abs(inputs3(48:72))) % 3rd MPC
peak_p = max(abs(u1(48*60:72*60)))% P-controller


%%% Total Comfort Violation Calculations %%%

% Upper violations of C2 Controller
upper_violations_mpc1 = find(Temps1(48:72)>T_ub);
upper_violations_mpc2 = find(Temps2(48:72)>T_ub);
upper_violations_mpc3 = find(Temps3(48:72)>T_ub);

% Upper violations of C1 Controller
upper_violations_p = find(Tc1(48*60:72*60)>T_ub);

% Lower violations of C2 controller
lower_violations_mpc1 = find(Temps1(48:72)<T_lb);
lower_violations_mpc2 = find(Temps2(48:72)<T_lb);
lower_violations_mpc3 = find(Temps3(48:72)<T_lb);

% Lower violations of C1 Controller
lower_violations_p = find(Tc1(48*60:72*60)<T_lb);

% Total comfort violation = Upper violations + Lower violations
comf_viol_mpc1 = sum((Temps1(48+upper_violations_mpc1-1)-T_ub)*dt) + sum(T_lb - (Temps1(48+lower_violations_mpc1-1))*dt)
comf_viol_mpc2 = sum((Temps2(48+upper_violations_mpc2-1)-T_ub)*dt) + sum(T_lb - (Temps1(48+lower_violations_mpc1-1))*dt)
comf_viol_mpc3 = sum((Temps3(48+upper_violations_mpc3-1)-T_ub)*dt) + sum(T_lb - (Temps1(48+lower_violations_mpc1-1))*dt)
comf_viol_p = sum((Tc1(48*60+upper_violations_p-1)-T_ub)*dt)/60 + sum(T_lb - (Tc1(48*60+lower_violations_p-1))*dt)/60

