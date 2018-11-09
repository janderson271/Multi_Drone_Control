function [u] = solve_cftoc_YALMIP(A, B, Bd, P, Q, R, N, xbar, ubar, x0, xL, xU, uL, uU, bf, Af)

[nx,nu] = size(B);

xOpt = sdpvar(nx,N+1);
uOpt = sdpvar(nu,N);

xbarm = xbar.*ones(nx,N);
ubarm = ubar.*ones(nu,N);

%% Cost 
JOpt =            (xOpt(:,end)-xbar)'*P*(xOpt(:,end)-xbar)       + ...
       trace((xOpt(:,1:end-1)-xbarm)'*Q*(xOpt(:,1:end-1)-xbarm)) + ...
       trace(           (uOpt-ubarm)'*R*(uOpt-ubarm)           ) ;

%% Constraints
C = [];  % initialize constraints array

% dynamic constraints
for k = 1:N
    C = [C,xOpt(:,k+1) == A*xOpt(:,k) + B*uOpt(:,k) + Bd];
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
    error('Infeasible Problem')
    return;
elseif soln.problem ~= 0
    error('YALMIP Problem')
    return;
end

u = double(uOpt(:,1));
end