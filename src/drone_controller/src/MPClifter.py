# MPC Controller

import numpy as np
import cvxpy as cvx
import scipy
from scipy import signal, misc

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
		self.n = n
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
		self.A = A

		# B discrete form
		B = np.zeros((12,4))
		B[5,:] = 1/m*np.array([1,1,1,1])
		B[9:12,:] = np.array([[l,-l,-l,l],[-l,-l,l,l],[k,-k,k,-k]])
		B[ 9,:] = B[ 9,:]/inertia_xx
		B[10,:] = B[10,:]/inertia_yy
		B[11,:] = B[11,:]/inertia_zz
		B = ts*B
		self.B = B

		# Dynamics Affine Disturbance discrete form
		Bd = np.zeros((12,1))
		Bd[5] = -g
		Bd = ts*Bd.flatten()
		self.Bd = Bd

		# Force Disturbance Parameter
		self.Fd = cvx.Parameter(3)
		self.Fd.value = np.zeros((3,1)).flatten()
		BFd = np.zeros((12,3))
		BFd[3,0] = ts/m 
		BFd[4,1] = ts/m
		BFd[5,2] = ts/m
		self.BFd = BFd

		# Disturbance Observer
		self.Cd = BFd.transpose()
		self.x_obs = np.zeros((15,1))
		self.A_obs = np.block([[A,BFd],[np.zeros((3,12)),np.eye(3)]])
		self.B_obs = np.block([[B],[np.zeros((3,4))]])
		self.Bd_obs = np.block([[Bd.reshape((12,1))],[np.zeros((3,1))]])
		self.C_obs = np.block([np.eye(12),np.zeros((12,3))])
		poles = np.array([93.,94,95,96,97,98,99,100,101,102,103,104,105,106,107])
		poles = poles/130
		sys = scipy.signal.place_poles(self.A_obs.transpose(),self.C_obs.transpose(),poles)
		self.L_obs = sys.gain_matrix.transpose()

		# MPC cost function
		xbar = cvx.Parameter(12)
		xbar.value = xbar_v.flatten()
		ubar = cvx.Parameter(4)
		ubar.value = ubar_v.flatten()

		# Variable constraints
		xl = [None]*nx
		xu = [None]*nx
		d = 15
		for i in range(6,8):
			xl[i] = -d*3.14/180
			xu[i] =  d*3.14/180
		dv = 10
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
			constraints += [X[:,k+1] == A*X[:,k] + B*U[:,k] + Bd + BFd*self.Fd]	

		# State Constraints
		constraints += [X[:,0] == self.x0] # Initial Constraint
		for k in range(4,n):
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

		self.counter = 1 # for when solver fails

	def actuate(self,x0):

		self.observe_Fd(x0)

		self.x0.value = x0.flatten()
		print(self.X[8,-1].value)

		#self.problem.solve()
		self.problem.solve(solver=cvx.CVXOPT)

		if self.problem.status == "optimal":
			if self.counter != 1:
				self.counter = 1
			return self.U[:,0].value
		else:
			if self.counter <= self.n:
				print(self.problem.status)
				self.counter += 1
				return self.U[:,self.counter-1].value
			else:
				print(self.problem.status)
				return None

	def change_xbar(self, xbar_new):
		self.xbar.value = xbar_new.flatten()

	def observe_Fd(self,x):
		if self.U[:,0].value is None:
			return

		yk = self.X[:,1].value.reshape((12,1))

		self.x_obs = np.dot(self.A_obs,self.x_obs) + \
					 np.dot(self.B_obs,self.U[:,0].value.reshape((4,1))) + \
					 self.Bd_obs + \
					 self.L_obs.dot(x.reshape((12,1)) - yk)

		self.Fd.value = self.x_obs[12:15].flatten()
		# print('Fd Disturbance')
		# print(self.Fd.value)
		return
		


