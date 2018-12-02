%% simulation test setup
ts = 0.1;

% drone characteristics
l = 0.033;
m = 0.032;
J = diag([16e-6,16e-6,29e-6]);

% drone dynamics
x0 = zeros(12,1);
g = 9.81;

nx = 12;
A = zeros(12,12);
A(1:3,4:6) = eye(3);
A(4,8) = g;
A(5,7) = -g;
A(7:9,10:12) = eye(3);
A = eye(12) + ts*A; % ZOH discretization

nu = 4;
k = 0.3;
B = zeros(12,4);
B(6,:) = 1/m*[1,1,1,1];
B(10:12,:) = inv(J)*[l,-l,-l,l;-l,-l,l,l;k,-k,k,-k];
B = ts*B; % ZOH discretization

Bd = zeros(12,1);
Bd(6) = -g;
Bd = ts*Bd; % ZOH discretization

%% Setup Problem
% Quadratic Cost Function
xbar = zeros(nx, 1);
xbar(1:3) = 1;
P    = eye(nx,nx);
Q    = eye(nx);
ubar = zeros(nu,1);
R    = eye(nu);

% Horizon
n = 15;

% state constraints
m = 100;
xL = -m*ones(12,1);
xU =  m*ones(12,1);
uL =   zeros( 4,1);
uU =  m*ones( 4,1);

% Ax==b
xf = zeros(12,1);
xf(3) = 1;
bf = [xf;-xf];
Af = [eye(12);-eye(12)];
Af = [];
bf = [];

%% Solve
hor = 30;
xSim = zeros(nx,hor+1);
xSim(:,1) = x0;

for t = 1:hor
    disp(t)
    [feas, xOpt, uOpt, JOpt] = solve_cftoc(A, B, Bd, P, Q, R, n, xbar, ubar, xSim(:,t), xL, xU, uL, uU, bf, Af);
    u = uOpt(:,1);
    xSim(:,t+1) = A*xSim(:,t) + B*u + Bd;
end


%% Plot
figure
scatter3(xSim(1,:), xSim(2,:), xSim(3,:))
xlim([-2,2])
ylim([-2,2])
zlim([0,2])

