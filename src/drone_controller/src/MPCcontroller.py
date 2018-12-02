# MPC Controller

import numpy as np
import cvxpy as cvx

class MPCcontroller():

	def __init__(self, ts = 0.1, P = np.ones((12,1)),   \
							     Q = np.ones((12,1)),   \
							     R = np.ones(( 4,1)),   \
						    xbar_v = np.zeros((12,1)),  \
					        ubar_v = np.zeros(( 4, 1)), \
					          x0_v = np.zeros((12, 1))  ):
			
		P = np.diag(P)
		Q = np.diag(Q)
		R = np.diag(R)

		l=0.033
		m=0.032
		inertia_xx = 16e-6
		inertia_yy = inertia_xx
		inertia_zz = 29e-6
		k=0.01

		n = 10
		g = 9.81

		self.x0 = cvx.Parameter(12)
		self.x0.value = x0_v.flatten()

		nx = 12
		nu = 4

		# A discrete form 
		A = np.zeros((12,12))
		A[0:3,3:6] = np.eye(3)
		A[3,7] = g
		A[4,6] = -g
		A[6:9,9:12] = np.eye(3)
		A = np.eye(12) + ts*A

		# B discrete form
		B = np.zeros((12,4))
		B[5,:] = 1/m*np.array([1,1,1,1])
		B[9:12,:] = np.array([[l,-l,-l,l],[-l,-l,l,l],[k,-k,k,-k]])
		B[ 9,:] = B[ 9,:]/inertia_xx
		B[10,:] = B[10,:]/inertia_yy
		B[11,:] = B[11,:]/inertia_zz
		B = ts*B

		# Dynamics Affine Disturbance discrete form
		Bd = np.zeros((12,1))
		Bd[5] = -g
		Bd = ts*Bd.flatten()

		# MPC cost function
		xbar = cvx.Parameter(12)
		xbar.value = xbar_v.flatten()
		ubar = cvx.Parameter(4)
		ubar.value = ubar_v.flatten()

		# Variable constraints
		xl = [None]*nx
		xu = [None]*nx
		v = 0.15
		for i in range(3,5):
			xl[i] = -v
			xu[i] =  v
		d = 15
		for i in range(6,8):
			xl[i] = -d*3.14/180
			xu[i] =  d*3.14/180
		dv = 20
		for i in range(9,11):
			xl[i] = -dv*3.14/180
			xu[i] =  dv*3.14/180

		ul =  np.zeros((4,1)).flatten()
		uu =  0.15*np.ones( 4).flatten()

		# CVXPY Formulation
		X = cvx.Variable((nx,n + 1))
		U = cvx.Variable((nu,n))

		# Cost function
		J = cvx.quad_form(X[:,n]-xbar,P)
		for k in range(n):
			J += cvx.quad_form(X[:,k]-xbar,Q) + cvx.quad_form(U[:,k]-ubar,R)

		constraints = []
		# Dynamic Constraints		
		for k in range(0,n):
			constraints += [X[:,k+1] == A*X[:,k] + B*U[:,k] + Bd]	

		# State Constraints
		constraints += [X[:,0] == self.x0] # Initial Constraint
		for k in range(1,n):
			for i in range(nx):
				if xl[i] is not None:
					constraints += [xl[i] <= X[i,k]]
				if xu[i] is not None:
					constraints += [X[i,k] <= xu[i]]

		# Input Constraints
		for k in range(0,n):
			for i in range(nu):
				if ul[i] is not None:
					constraints += [ul[i] <= U[i,k]]
				if uu[i] is not None:
					constraints += [U[i,k] <= uu[i]]

		self.X = X
		self.U = U
		self.xbar = xbar
		self.ubar = ubar
		self.problem = cvx.Problem(cvx.Minimize(J),constraints)

	def actuate(self,x0):
		self.x0.value = x0.flatten()
		#self.problem.solve()
		self.problem.solve(solver=cvx.CVXOPT)
		print('Final xOpt')
		print(self.X.value[:,-1])
		if self.problem.status == "optimal":
			return self.U[:,0].value
		else:
			print(self.problem.status)
			return None
