# test script for CVXPY 

import numpy as np
import cvxpy as cvx
import matlab.engine

class Test():

	def __init__(self):

		self.ts = 0.1
		ts = 0.1

		# drone characteristics
		l = 1
		w = 1
		m = 1
		J = np.diag([0.45,0.45,0.7])

		# drone dynamics
		self.x0 = np.zeros((12,1)).flatten()
		g = 9.81

		self.nx = 12
		A = np.zeros((12,12))
		A[0:3,3:6] = np.eye(3)
		A[3,7] = g
		A[4,6] = -g
		A[6:9,9:12] = np.eye(3)
		self.A = np.eye(12) + ts*A # ZOH discretization

		self.nu = 4
		k = 1
		B = np.zeros((12,4))
		B[5,:] = 1/m*np.array([1,1,1,1])
		B[9:12,:] = np.array([[l,-l,-l,l],[-l,-l,l,l],[-k,k,-k,k]])
		self.B = ts*B # ZOH discretization

		Bd = np.zeros((12,1))
		Bd[5] = -g
		self.Bd = (ts*Bd).flatten() # ZOH discretization

		# Quadratic Cost Function
		self.xbar = np.zeros((self.nx, 1))
		self.P    = np.zeros((self.nx,self.nx))
		self.Q    = np.zeros((self.nx,self.nx))
		self.ubar = np.zeros((self.nu,1))
		self.R    = np.zeros((self.nu,self.nu))
		
		# Horizon
		self.n = 10

		# state constraints
		m = 100
		self.xL = -m*np.ones(12).flatten()
		self.xU =  m*np.ones(12).flatten()
		self.uL = -m*np.ones( 4).flatten()
		self.uU =  m*np.ones( 4).flatten()

		# terminal constraints
		self.bf = np.zeros((self.nx,1))
		self.Af = np.identity(self.nx)

		# Ax==b
		xf = np.zeros((12,1))
		self.bf = np.vstack((xf,-xf)).flatten()
		self.Af = np.vstack((np.eye(12),-np.eye(12)))


	def solve_cftoc_CVXPY(self):

		X = cvx.Variable((self.nx,self.n + 1))
		U = cvx.Variable((self.nu,self.n))

		# Cost function
		J = cvx.quad_form(X[:,self.n],self.P)
		for k in range(self.n):
			J += cvx.quad_form(X[:,k],self.Q) + cvx.quad_form(U[:,k],self.R)

		constraints = []
		# Dynamic Constraints
		for k in range(0,self.n):
			constraints += [X[:,k+1] == self.A*X[:,k] + self.B*U[:,k] +self.Bd]	

		# State Constraints
		constraints += [X[:,0] == self.x0] # Initial Constraint
		for k in range(1,self.n):
			constraints += [self.xL <= X[:,k], X[:,k] <= self.xU]
		constraints += [self.Af*X[:,self.n] <= self.bf] # Terminal Constraint

		# Input Constraints
		for k in range(0,self.n):
			constraints += [self.uL <= U[:,k], U[:,k] <= self.uU]

		# Solve Program
		problem = cvx.Problem(cvx.Minimize(J),constraints)
		solution = problem.solve()

		return U[:,0].value

	def solve_cftoc_YALMIP(self):
		eng = matlab.engine.start_matlab()
		eng.addpath(r'~/Documents/MATLAB/YALMIP-master',nargout=0)
		eng.addpath(r'~/Documents/MATLAB/YALMIP-master/demos',nargout=0)
		eng.addpath(r'~/Documents/MATLAB/YALMIP-master/extras',nargout=0)
		eng.addpath(r'~/Documents/MATLAB/YALMIP-master/modules',nargout=0)
		eng.addpath(r'~/Documents/MATLAB/YALMIP-master/operators',nargout=0)
		eng.addpath(r'~/Documents/MATLAB/YALMIP-master/@sdpvar',nargout=0)
		eng.addpath(r'~/Documents/MATLAB/YALMIP-master/solvers',nargout=0)

		return eng.solve_cftoc_YALMIP( matlab.double(self.A.tolist()),                         \
									   matlab.double(self.B.reshape((self.nx,self.nu)).tolist()),    \
									   matlab.double(self.Bd.reshape((self.nx,1)).tolist()),   \
									   matlab.double(self.P.tolist()),   					   \
									   matlab.double(self.Q.tolist()),                         \
									   matlab.double(self.R.tolist()),                         \
		 							   matlab.double(np.array([self.n]).tolist()),           \
		 							   matlab.double(self.xbar.reshape((self.nx,1)).tolist()), \
		 							   matlab.double(self.ubar.reshape((self.nu,1)).tolist()), \
		 							   matlab.double(self.x0.reshape((self.nx,1)).tolist()),   \
		 							   matlab.double(self.xL.reshape((self.nx,1)).tolist()),   \
		 							   matlab.double(self.xU.reshape((self.nx,1)).tolist()),   \
		 							   matlab.double(self.uL.reshape((self.nu,1)).tolist()),   \
		 							   matlab.double(self.uU.reshape((self.nu,1)).tolist()),   \
		 							   matlab.double(self.bf.reshape((2*self.nx,1)).tolist()),   \
		 							   matlab.double(self.Af.tolist()))	

	def solve_cftoc_OS(self,A, B, N, Q, R, P, x0, umax=None, umin=None, xmin=None, xmax=None):
	    """
	    solve MPC with modeling tool for test
	    """
	    import cvxpy as cvxpy

	    (nx, nu) = B.shape
	    B = B.flatten()

	    # mpc calculation
	    x = cvxpy.Variable((nx, N + 1))
	    u = cvxpy.Variable((nu, N))

	    costlist = 0.0
	    constrlist = []

	    for t in range(N):
	        costlist += 0.5 * cvxpy.quad_form(x[:, t], Q)
	        costlist += 0.5 * cvxpy.quad_form(u[:, t], R)

	        constrlist += [x[:, t + 1] == A * x[:, t] + B * u[:, t]]

	        if xmin is not None:
	            constrlist += [x[:, t] >= xmin]
	        if xmax is not None:
	            constrlist += [x[:, t] <= xmax]

	    costlist += 0.5 * cvxpy.quad_form(x[:, N], P)  # terminal cost
	    if xmin is not None:
	        constrlist += [x[:, N] >= xmin]
	    if xmax is not None:
	        constrlist += [x[:, N] <= xmax]

	    #import ipdb; ipdb.set_trace()
	    constrlist += [x[:, 0] == x0]  # inital state constraints
	    if umax is not None:
	        constrlist += [u <= umax]  # input constraints
	    if umin is not None:
	        constrlist += [u >= umin]  # input constraints

	    prob = cvxpy.Problem(cvxpy.Minimize(costlist), constrlist)
	    prob.solve(verbose=False)

	    return u[:,0].value

if __name__ == "__main__":
	test = Test()

	u_CVXPY = test.solve_cftoc_CVXPY()
	print(u_CVXPY)	

	u_YALMIP = test.solve_cftoc_YALMIP()
   	print(u_YALMIP)	

   	#u_OS = test.solve_cftoc_OS(test.A, test.B.reshape((test.nx,test.nu)), test.n, test.Q, test.R, test.P, test.x0, \
   	#								umax=test.uU[0], umin=test.uL[0], \
   	#								xmin=test.xL[0], xmax=test.xU[0])
   	#print(u_OS)

