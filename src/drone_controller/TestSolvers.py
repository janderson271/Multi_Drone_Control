# test script for CVXPY 

import numpy as np
import cvxpy as cvx
import matlab.engine

class Test():

	def __init__(self):

		self.ts = 0.1
		self.x0 = np.array([[2],[-1]]).flatten()

		#dynamics
		self.nx = 2
		self.nu = 1
		self.A = np.array([[1.2,1],[0,1]])
		self.B = np.array([[0],[1]]).flatten()
		self.Bd = np.zeros((2,1)).flatten()

		# Quadratic Cost Function
		self.xbar = np.array([[0],[0]])
		self.P    = np.zeros((2,2))
		self.Q    = np.eye(2)
		self.ubar = np.array([[0]])
		self.R    = np.array([[1]])
		
		# Horizon
		self.n = 25

		# state constraints
		self.xL = np.array([-15,-15])
		self.xU = np.array([15,15])
		self.uL = np.array([-1])
		self.uU = np.array([1])

		# Ax<=b
		#self.bf = np.zeros((2*self.nx,1)).flatten()
		#self.Af = np.array([[1,0],[0,1],[1,0],[0,1]])

		# Ax==b
		self.bf = np.zeros((2*self.nx,1)).flatten()
		self.Af = np.array([[1,0],[0,1],[-1,0],[0,-1]])

		# no constraint
		#self.bf = np.zeros((2*self.nx,1)).flatten()
		#self.Af = np.zeros((4,2))



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
			constraints += [X[:,k+1] == self.A*X[:,k] + self.B*U[:,k]]	

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
									   matlab.double(self.B.reshape((self.nx,1)).tolist()),    \
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

	def use_modeling_tool(self,A, B, N, Q, R, P, x0, umax=None, umin=None, xmin=None, xmax=None):
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

   	u_OS = test.use_modeling_tool(test.A, test.B.reshape((test.nx,test.nu)), test.n, test.Q, test.R, test.P, test.x0, \
   									umax=test.uU[0], umin=test.uL[0], \
   									xmin=test.xL[0], xmax=test.xU[0])
   	print(u_OS)

