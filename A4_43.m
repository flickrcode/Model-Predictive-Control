clear;clc;
%% Q1-Q3: Explain and answer in the report
%% Q4: Fill in N1_XX.m and Ninf_XX.m in which XX is your group ID.
A = [2 5;0 1];
b = [1;1];

% First objective function output
x_1=N1_43(A,b)

% Second objective function output
x_inf=Ninf_43(A,b)

%% Q5: Provide your results and explanations in the report. Your code should come here to support 
% your results in the report
x0 = 2;
Aeq = [1 0 -1 0;0.5 0 0 1]
beq = [1;0.5]

H = eye(4)

% Switch the button to b and c for the different cases
button = 'c'
switch button
    case 'a'
        lb = [2.5;-1;-2;-2]
        ub = [5;1;2;2]
        x = quadprog(H,[],[],[],Aeq,beq,lb,ub,x0)
        
        states = x(1:2,:)
        inputs = x(3:4,:)

[x,fval,exitflag,output,lambda]= quadprog(H,[],[],[],Aeq,beq,lb,ub,x0)
 mu_lowerbound = lambda.lower
 mu_upperbound = lambda.upper
 lambda = lambda.eqlin

% if the lower bound of x1 is removed
       case 'b'
       lb = [-1;-2;-2] 
       ub = [5;1;2;2]
       x = quadprog(H,[],[],[],Aeq,beq,lb,ub,x0)
       
           states = x(1:2,:)
           inputs = x(3:4,:)

           [x,fval,exitflag,output,lambda]= quadprog(H,[],[],[],Aeq,beq,lb,ub,x0)
           mu_lowerbound = lambda.lower
           mu_upperbound = lambda.upper
           lambda = lambda.eqlin
           
% if the upper bound of x1 is removed
       case 'c'
       lb = [2.5;-1;-2;-2]    
       ub = [1;2;2]
       x = quadprog(H,[],[],[],Aeq,beq,lb,ub,x0)
       
           states = x(1:2,:)
           inputs = x(3:4,:)

           [x,fval,exitflag,output,lambda]= quadprog(H,[],[],[],Aeq,beq,lb,ub,x0)
           mu_lowerbound = lambda.lower
           mu_upperbound = lambda.upper
           lambda = lambda.eqlin
end




