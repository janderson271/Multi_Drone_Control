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

		self.r = xbar_v[0:3].reshape((3,1))

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
		Bg= np.zeros((12,1))
		Bg[5] = -g
		Bg= ts*Bg.flatten()
		self.Bg= Bg

		# Force Disturbance Parameter
		self.Fd = cvx.Parameter(3)
		self.Fd.value = np.zeros((3,1)).flatten()
		Bd = np.zeros((12,3))
		Bd[3,0] = ts/m 
		Bd[4,1] = ts/m
		Bd[5,2] = ts/m
		self.Bd = Bd

		# Disturbance Observer 
		self.x_obs = np.zeros((15,1))
		C = np.eye(12)
		Cd = np.zeros((12,3))
		self.A_obs = np.block([[A,Bd],[np.zeros((3,12)),np.eye(3)]])
		self.B_obs = np.block([[B],[np.zeros((3,4))]])
		self.Bg_obs = np.block([[Bg.reshape((12,1))],[np.zeros((3,1))]])
		self.C_obs = np.block([[C,Cd]])
		poles = np.array([93.,94,95,96,97,98,99,100,101,102,103,104,105,106,107])/150
		sys = scipy.signal.place_poles(self.A_obs.transpose(),self.C_obs.transpose(),poles)
		self.L_obs = sys.gain_matrix.transpose()
		self.R = R

		# Disturbance Rejection
		C = np.block([np.eye(3),np.zeros((3,9))])
		self.A_track = np.block([[A-np.eye(12),B],[C,np.zeros((3,4))]])

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
			constraints += [X[:,k+1] == A*X[:,k] + B*U[:,k] + Bg+ Bd*self.Fd]	

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

	def change_ubar(self, ubar_new):
		self.ubar.value = ubar_new.flatten()

	def observe_Fd(self,x):
		if self.U[:,0].value is None:
			return

		#xhat = self.X[:,1].value.reshape((12,1))

		self.x_obs = np.dot(self.A_obs,self.x_obs) + \
					 np.dot(self.B_obs,self.U[:,0].value.reshape((4,1))) + \
					 self.Bg_obs - \
					 self.L_obs.dot(self.x_obs[0:12] - x.reshape((12,1)))

		dhat = self.x_obs[12:15]
		self.Fd.value = dhat.flatten()

		print('Fd is observed')
		print(self.Fd.value)

		# solve for xtrack and utrack
		xtut = cvx.Variable((16,1)) # xt stacked on ut
		J = cvx.quad_form(xtut[12:16],self.R)
		constraints = [self.A_track*xtut + np.block([[self.Bg.reshape((12,1))],[np.zeros((3,1))]]) == np.block([[-self.Bd.dot(dhat)],[self.r]])]
		problem = cvx.Problem(cvx.Minimize(J),constraints)
		problem.solve()

		xt = xtut[0:12].value
		ut = xtut[12:16].value

		print('x')
		print(self.X[6:8,0].value)
		print(xtut[6:8].value)

		self.change_xbar(xt)
		self.change_ubar(ut)

		return
		


