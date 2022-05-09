function [u0,T1] = CRHC7_43(A,B,d,N,kappa,rho,T_ub,T_lb,T0)

%%%%%%%%%%    States to solve for are %%%%%%%%%%%%%
%%%%%%%%%%%%[z1;z2;z3;u;eps_up;eps_lo;T] %%%%%%%%%%

%%%% Define the minimization Problem %%%%
f = [ones(1,N) kappa rho*(ones(1,N)) zeros(1,N) zeros(1,N) zeros(1,N) zeros(1,N)];

%%%% Define Inequality constraints template %%%%
A1  = [-eye(N)  zeros(N,1) zeros(N)  -eye(N)  zeros(N) zeros(N) zeros(N)];
A2  = [-eye(N)  zeros(N,1) zeros(N)  eye(N)   zeros(N) zeros(N) zeros(N)];
A3  = [zeros(N) -ones(N,1) zeros(N)  -eye(N)  zeros(N) zeros(N) zeros(N)];
A4  = [zeros(N) -ones(N,1) zeros(N)  eye(N)   zeros(N) zeros(N) zeros(N)];
A5  = [zeros(N) zeros(N,1) -eye(N)   zeros(N) eye(N)   eye(N)   zeros(N)];
A6  = [zeros(N) zeros(N,1) -eye(N)   zeros(N) -eye(N)  -eye(N)  zeros(N)];
A7  = [zeros(N) zeros(N,1) zeros(N)  zeros(N) -eye(N)  zeros(N) eye(N)];
A8  = [zeros(N) zeros(N,1) zeros(N)  zeros(N) zeros(N) -eye(N)  -eye(N)];
A9  = [zeros(N) zeros(N,1) zeros(N)  zeros(N) -eye(N)  zeros(N) zeros(N)];
A10 = [zeros(N) zeros(N,1) zeros(N)  zeros(N) zeros(N) -eye(N)  zeros(N)];

%%% Aineq*x < bineq
Aineq = [A1;A2;A3;A4;A5;A6;A7;A8;A9;A10];
bineq = [zeros(N,1); zeros(N,1); zeros(N,1); zeros(N,1); zeros(N,1); zeros(N,1);...
           T_ub*ones(N,1); -T_lb*ones(N,1); zeros(N,1); zeros(N,1)];

%%%% Equality constraints %%%%
F = -B*eye(N);
G = zeros(N);
for i=1:N
    if i ==1
        G(i,i) = 1;
    else
        G(i,i-1:i) = [-A 1];
    end
end

%%%%%%%% Aeq*x = beq %%%%%%%%
Aeq = [zeros(N,2*N+1) F zeros(N,2*N) G];
beq = d; %zeros(N,1);
beq(1) = beq(1) + A*T0;

%%%% problem solution using the Linear Programming solver %%%%
[sol,fval] = linprog(f',Aineq,bineq,Aeq,beq);

inputs = sol(2*N+2:3*N+1);
Temps = sol(5*N+2:6*N+1);

u0 = inputs(1);
T1 = Temps(1);

end

