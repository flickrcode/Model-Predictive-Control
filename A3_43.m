clear;
close all;
clc;
%% Question 1
A=diag([0.5 0.6 0.5 0.6]);
B=[diag([0.5 0.4]);diag([0.25 0.6])];
C=[1 1 0 0;0 0 1 1];
z_sp=[1;-1];
 
 % Define the matrix for both xs and us
matrix_1=inv([eye(size(A,1))-A, -B; C, zeros(2,2)])*[0;0;0;0;z_sp]
 
% calculate the steady state (xs,us) and define them as follows; Do not change
% the names of variables; Report these values in the report
 
% Separate the result above into xs,us
xs_1=matrix_1(1:4)
us_1=matrix_1(5:6)

xp=A*xs_1+B*us_1;
y=C*xp
 
%% Question 2
A=diag([0.5 0.6 0.5 0.6]);
B=[0.5;0;0.25;0];
C=[1 1 0 0;0 0 1 1];
y_sp=[1;-1];
% calculate the steady state (xs,us) and define them as follows; Do not change
% the names of variables; Report these values in the report
 
H=[1,0;0,1];
x = optimvar('x',5)
showvar(x)
showbounds(x)
a= C*x(1:4)-y_sp
showexpr(a)

objective = a'*H*a
showexpr(objective)
prob = optimproblem('Objective',objective);
prob.Constraints.cons1 = 0.5*x(1)-0.5*x(5) == 0 ;
prob.Constraints.cons2 = 0.4*x(2) == 0;
prob.Constraints.cons3 = 0.5*x(3) - 0.25*x(5) == 0;
prob.Constraints.cons4 = 0.4*x(4)  == 0;
prob.Constraints.cons5 = x(1)+x(2) >= 1;
prob.Constraints.cons6 = x(3)+x(4) <= -1;
 showproblem(prob)
problem = prob2struct(prob);
x = quadprog(problem);
 
% Separate the result into xs and us and check if the output is
% offset-free
xs_2=x(1:4)
us_2=x(5)

xp=A*xs_2+B*us_2;
y=C*xp

%% Question 3
A=diag([0.5 0.6 0.5 0.6]);
B=[diag([0.5 0.4]);diag([0.25 0.6])];
C=[1 1 0 0];
y_sp=1;
 
% calculate the steady state (xs,us) and define them as follows; Do not change
% the names of variables; Report these values in the report
 
Q=1;
R=1;
us=[1;1];
x = optimvar('x',6);

% Define A and B as the 
A=x(5:6)-us
B=C*x(1:4)-y_sp
% showexpr(B)

objec =A'*Q*A + B'*R*B;
prob = optimproblem('Objective',objec);
prob.Constraints.cons1 = 0.5*x(1)- 0.5*x(5) == 0 ;
prob.Constraints.cons2 = 0.4*x(2) - 0.4*x(6) == 0;
prob.Constraints.cons3 = 0.5*x(3) - 0.25*x(5) == 0;
prob.Constraints.cons4 = 0.4*x(4) - 0.6*x(6) == 0;
prob.Constraints.cons5 = x(1) == 0;
prob.Constraints.cons6 = x(1)+x(2) >= 1;
problem = prob2struct(prob);
x = quadprog(problem)

 % Separate the result into xs and us and check if the output is
% offset-free
xs_3= x(1:4)
us_3=x(5:6)

AA = diag([0.5 0.6 0.5 0.6]);
BB = [diag([0.5 0.4]);diag([0.25 0.6])];
CC = [1 1 0 0];

xp=AA*xs_3+BB*us_3;
y=CC*xp

%% second part
tf=50;                
%==========================================================================
% Process model
%==========================================================================
 
h = 1; %sampling time in minutes
 
A = [ 0.2681   -0.00338   -0.00728;
      9.703    0.3279   -25.44;
         0         0       1   ];
B = [ -0.00537  0.1655;
       1.297   97.91 ;
       0       -6.637];
C = [ 1 0 0;
      0 1 0;
      0 0 1];
Bp = [-0.1175;
      69.74;
       6.637 ];
   
n = size(A,1); % n is the dimension of the state
m = size(B,2); % m is the dimension of the control signal
p = size(C,1); % p is the dimension of the measured output
 
d=0.01*[zeros(1*tf/5,1);ones(4*tf/5,1)]; %unmeasured disturbance trajectory
 
x0 = [0.01;1;0.1]; % initial condition of system's state
 
%==========================================================================
% Observer model
%==================================e_========================================
%% choose one case, i.e. a, b, or c and then write the code for that case! for the other ones
% you just need to change the example case!
example = 'a';

switch example
    case 'a'
        nd = 2;
        Bd = zeros(n,nd);
        Cd = [1 0;0 0; 0 1]; 
        A_aug_1=[A, Bd;zeros(2,3) eye(2)]
    case 'b'
        nd=3;
        Bd = zeros(n,nd); 
        Cd = [1 0 0;0 0 1;0 1 0];
        A_aug_2=[A, Bd;zeros(3) eye(3)]
    case 'c'
        nd=3;
        Bd = [zeros(3,2) Bp];
        Cd = [1 0 0;0 0 0;0 1 0];
        A_aug_3=[A, Bd;zeros(3) eye(3)]
end
 
 
%% Question 4
%Augment the model with constant disturbances; check the detectability for case "a", "b", and "c" and
%report the detectability of each one in the report
 
% define Ae, Be, and Ce which are the matrices for the augmented system. No
% need to report these in the report
%  

% Do the Hautus Detectability test for each cases
switch example
    case 'a'
Ae = A_aug_1
Be = [B;zeros(2)]
Ce = [C Cd]
lambda_all_1 = eig(Ae)
lambda_non_negative_1 = real(lambda_all_1)
Detectability_test_1 = [lambda_non_negative_1.*eye(5)-Ae;Ce] 

if rank(Detectability_test_1) == length(Ae)
   Detectability_1 = 1 
    disp('system is detectable')
else Detectability_1 = 0
    disp('system is not detectable')
end
  case 'b'
Ae = A_aug_2
Be = [B;zeros(1,2)]
Ce = [C Cd]
lambda_all_2 = eig(Ae)
lambda_non_negative_2 = real(lambda_all_2)
Detectability_test_2 = [lambda_non_negative_2.*eye(6)-Ae;Ce] 

  if rank(Detectability_test_2) == length(Ae)
   Detectability_2 = 1 
    disp('system is detectable')
else Detectability_2 = 0
    disp('system is not detectable')
  end
  
 case 'c'
Ae = A_aug_3
Be = [B;zeros(3,2)]
Ce = [C Cd]
lambda_all_3 = eig(Ae)
lambda_non_negative_3 = real(lambda_all_3)
Detectability_test_3 = [lambda_non_negative_3.*eye(6)-Ae;Ce]

  if rank(Detectability_test_3) == length(Ae)
   Detectability_3 = 1 
    disp('system is detectable')
else Detectability_3 = 0
    disp('system is not detectable')
  end
end

 
%% Question 5
% Calculate Kalman filter gain and name it Le; no need to report this in
% the report

% Define the Covariance matrix and its scaling
 w = [1 1 1];
 scale1 = 1;
Q_w = scale1*diag(w);

scale2 = 1;
v = [1 1 1];
Q_v = scale2*diag(v);
 
% Find the Kalman Filter Gain for each corresponding cases
 switch example
      case 'a'
          De = [0;0;0]
          G = [0 0; 0 0; 0 0; 1 0; 0 1];
          He = zeros(3);
          Nn = zeros(3);
           
        G_Delta_kalman_1 = ss(Ae,[Be G],Ce,[De He])
        [kest1,Le,P] = kalman(G_Delta_kalman_1,Q_w,Q_v,Nn)
        Le_state = Le(1:3,:)
        Le_disturbance = Le(4:5,:)
      case 'c'
          De = [0;0;0]
          G = [0 0; 0 0; 0 0; 0 0; 1 0; 0 1];
           He = zeros(3);
          Nn = zeros(3);
          
        G_Delta_kalman_3 = ss(Ae,[Be G],Ce,[De He])
        [kest3,Le,P] = kalman(G_Delta_kalman_3,Q_w,Q_v,Nn)
        Le_state = Le(1:3,:)
        Le_disturbance = Le(4:5,:)
 end
 
 
 
%% Question 6
%Target Selector
A = [ 0.2681   -0.00338   -0.00728;
      9.703    0.3279   -25.44;
         0         0       1   ];
B = [ -0.00537  0.1655;
       1.297   97.91 ;
       0       -6.637];
 H = [1 0 0;0 0 1]; 
 
 Cs = H*C
 xsus6=inv([eye(size(A,1))-A, -B; Cs, zeros(2,2)])
 
 switch example
      case 'a'
 % Matrices for steady state target calculation of 1st detectable augmented
 % system
 
  Mss_1 = xsus6*[Bd;-H*Cd]
  
  case 'c'
 % Matrices for steady state target calculation of 3rd detectable augmented
 % system
 Mss_3 = xsus6*[Bd;-H*Cd]
  end

%note that Mss is defined by [xs;us]=Mss*d and you will use it later on; no need to report this in
% the report
 
%% Question 7
%==========================================================================
% Setup MPC controller
%==========================================================================
 
sp=zeros(tf,3);         % setpoint trajectory

 
N=10;                   % prediction horizon
M=3;                    % control horizon
 
Q = diag([1 0.001 1]);  % state penalty
Pf = Q;                 % terminal state penalty
R = 0.01*eye(m);         % control penalty

xhat_ = zeros(n, tf); %Initial state estimation definition
dhat_ = zeros(nd, tf); %Initial disturbance definition
xhat = xhat_; 
dhat = dhat_;
x = zeros(n, tf); 
v = zeros(p, tf);
y = zeros(p, tf);


%==========================================================================
% Simulation
%==========================================================================
    
% Simulation
    

    for k = 1:tf
        %=============================
        % Calculate steady state target
        %=============================
       switch example
      case 'a'   
 target_1.xsus = Mss_1*d(k)
%  target_3.xsus = Mss_3*d(k)

target_1.xs_1 = target_1.xsus(1:3,:)
 target_1.us_1 = target_1.xsus(4:5,:)
 
xp=A*target_1.xs_1+B*target_1.us_1;
target_1.ys=Cs*xp
         
        %=============================
        % Solve the QP by using the modified Batch Approach
        %=============================

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
x0=[0.01;1;0.1];
u=-inv(G'*Qbar*G+Rbar)*G'*Qbar*O*x0(:,1);

        %=============================
        % Update the observer state
        %=============================
%         
%   % measurement
  y(:,k) = C*x(:,k) + v(:,k);
  
  % state estimate
  ey = y(:,k) - C*xhat_(:,k) -Cd*dhat_(:,k);
  xhat(:,k) = xhat_(:,k) + Le_state*ey;
  dhat(:,k) = dhat_(:,k) + Le_disturbance*ey;
  
  H  = [1 0 0; 0 0 1];
  G = [eye(n)-A, -B; H*C, zeros(size(H,1), m)];
  qs = Mss_1*dhat(:,k)
  
  xss = qs(1:n); 
  uss = qs(n+1:end);
% 
  xs(:,k) = xss;
  us(:,k) = uss;
  ys(:,k) = C*xss + Cd*dhat(:,k);
  
   % control law
  control.x0 = xhat(:,k) - xs(:,k);
  
  % LQ controller
  u(:,k) = K0*control.x0 + us(:,k);
  
  if (k == ntimes) 
      break; 
  end
%         
%         %=============================
%         % Update the process state
%         %=============================
         % advance process state estimates
  xhat_(:,k+1) = A*xhat(:,k) + Bd*dhat(:,k) + B*u(:,k);
  dhat_(:,k+1) = dhat(:,k);
        
%         %=============================        
%         % Store current variables in log 
        %=============================
       end
   end % simulation loop
        

        %%
%==========================================================================
% Plot results
%==========================================================================
        
   % plot the states, the state estimations, and the input and report them
   % in the report.
    
