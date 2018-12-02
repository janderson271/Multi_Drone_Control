function [feas, xOpt, uOpt, JOpt] = solve_cftoc(A, B, Bd, P, Q, R, N, xBar, uBar, x0, ...
    xL, xU, uL, uU, bf, Af)
n = length(x0);
m = length(uL);
z = sdpvar(n,N+1);
u = sdpvar(m,N);

Cost = 0;
Constraints = [z(:,1) == x0];
for i = 1:N
    Cost = Cost + (z(:,i)- xBar)'*Q*(z(:,i)- xBar) + (u(:,i) - uBar)'*R*(u(:,i) - uBar);
    Constraints = [Constraints, z(:,i+1) == A*z(:,i) + B*u(:,i) + Bd, ...
        xL <= z(:,i), z(:,i) <= xU, ...
        uL <= u(:,i), u(:,i) <= uU];
end

if (isempty(Af) && ~isempty(bf)) 
    Constraints = [Constraints, z(:,N+1) == bf];
elseif (~isempty(Af))
    Constraints = [Constraints, Af*z(:,N+1) <= bf];
end

Cost = Cost +(z(:,N+1)- xBar)'*Q*(z(:,N+1)- xBar);

options = sdpsettings('verbose', 0, 'solver', 'quadprog'); 
sol = optimize(Constraints, Cost, options);

if sol.problem == 1
    feas = 0;
    xOpt = [];
    uOpt = [];
    JOpt = NaN;
else
    feas = 1;
    xOpt = double(z);
    uOpt = double(u);
    JOpt = double(Cost);
end

end

