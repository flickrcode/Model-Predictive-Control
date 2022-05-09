function [x,fval]=N1_43(A,b)
%% This function finds x to minimize the first norm
%% The inputs are Matrix a and vector b
%% The outputs are the optimal x and the norm value (fval)
%% Do not change the inputs and outputs!
%% use linprog to solve the problem
A = [2 5;0 1];
b = [1;1];

% Determine the row size of matrix A and b
m = size(A,1); 
n = size(b,1);

% Determine the objective function
 f = [ zeros(n,1) ones(n,1)];
 
 % Create an augmented matrix
 A_aug = [ +A, -eye(m) ; -A, -eye(m) ];
 b_aug = [ +b; -b ];
 
  % Use Linear Programming
solution = linprog(f,A_aug,b_aug);

%  % Optimal solution x
  x = solution(1:n,:);
 

end
