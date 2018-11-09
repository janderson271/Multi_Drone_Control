function [UU] = solve_cftoc(A, B, P, Q, R, N, x0, xbar, ubar, xL, xU, uL, uU, bf, Af)

if isempty(xbar)
    xbar = 0*x0;
end
if isempty(ubar)
    ubar = 0*uU;
end

nx = size(A,2);
nu = size(B,2);

xOpt = sdpvar(nx,N+1);
uOpt = sdpvar(nu,N);

%% Cost 
JOpt = (xOpt(:,end)-xbar)'*P*(xOpt(:,end)-xbar) + ...
        trace((xOpt(:,1:end-1)-xbar.*ones(nx,N))'*Q*(xOpt(:,1:end-1)-xbar.*ones(nx,N))) + ...
        trace((uOpt-ubar.*ones(nu,N))'*R*(uOpt-ubar.*ones(nu,N)));

%% Constraints
C = [];  % initialize constraints array

% dynamic constraints
for k = 1:N
    C = [C,xOpt(:,k+1) == A*xOpt(:,k) + B*uOpt(:,k)];
end

% state constaints
C = [C,xOpt(:,1) == x0];
for k = 2:N
    C = [C, xL <= xOpt(:,k) <= xU];
end
if isempty(bf)
elseif isempty(Af)
    C = [C,xOpt(:,end)==bf];
else
    C = [C,Af*xOpt(:,end) <= bf];
end

% input constraints
for k = 1:N
    C = [C, uL <= uOpt(:,k) <= uU];
end

%% Solve

options = sdpsettings('solver','quadprog','verbose',0);

soln = optimize(C,JOpt,options);

if soln.problem == 1
    feas = 0;
    xOpt = [];
    uOpt = [];
    JOpt = [];
    return;
end

feas = 1;
xOpt = double(xOpt);
uOpt = double(uOpt);
JOpt = double(JOpt);

UU = uOpt(:,1);

end