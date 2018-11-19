ts = 0.1;

% drone characteristics
l = 0.5;
m = 1;
J = diag([0.45,0.45,0.7]);

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
B(10:12,:) = inv(J)*[l,-l,-l,l;-l,-l,l,l;-k,k,-k,k];
B = ts*B; % ZOH discretization

Bd = zeros(12,1);
Bd(6) = -g;
Bd = ts*Bd; % ZOH discretization

% Quadratic Cost Function
xbar = zeros(nx, 1);
xbar(3) = 1;
P    = zeros(nx,nx);
Q    = eye(nx);
ubar = zeros(nu,1);
R    = eye(nu);

% Horizon
n = 10;

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

[u] = solve_cftoc_YALMIP(A, B, Bd, P, Q, R, n, xbar, ubar, x0, xL, xU, uL, uU, bf, Af)