# MPC Controller

import numpy as np
import cvxpy as cvx

class MPCcontroller():

	def __init__(self, ts = 0.1, x0=np.zeros((12,1)).flatten(), l=0.5, m=1, inertia_xx=.45, inertia_yy =0.45, inertia_zz=0.7, k=0.3):
		n = 10
		g = 9.81

		self.x0 = cvx.Parameter(12)
		self.x0.value = x0

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
		B[9:12,:] = np.array([[l,-l,-l,l],[-l,-l,l,l],[-k,k,-k,k]])
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
		xbar.value = np.zeros((nx, 1)).flatten()
		xbar.value[2] = 1
		P    = np.eye(nx)
		Q    = np.eye(nx)
		ubar = cvx.Parameter(4)
		ubar.value = np.zeros((nu,1)).flatten()
		R    = np.eye(nu)

		xl = [None] * nx
		xu = [None] * nx
#		ul = [None] * nu
		uu = [None] * nu

#		m = 100
#		xl = -m*np.ones((12,1)).flatten()
#		xu =  m*np.ones(12).flatten()
		ul =  np.zeros((4,1)).flatten()
#		uu =  5*np.ones( 4).flatten()

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
		constraints += [X[2,n] == 1] # Terminal Constraint

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
		self.problem.solve(solver = cvx.CVXOPT)
		if self.problem.status == "optimal":
			return self.U[:,0].value
		else:
			print(self.problem.status)
			return None