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

		self.ts = ts

		self.r = xbar_v[0:3].reshape((3,1))

		self.P = np.eye(12)#np.diag(P)
		self.Q = np.eye(12)#np.diag(Q)
		self.R = np.eye(4)#np.diag(R)

		l=0.033
		self.l = l
		m=0.032
		self.m = m
		self.inertia_xx = 16e-6
		self.inertia_yy = self.inertia_xx
		self.inertia_zz = 29e-6
		k = 0.01
		self.k=k

		self.n = 10
		self.g = 9.81

		self.x0 = cvx.Parameter(12)
		self.x0.value = x0_v.flatten()

		self.nx = 12
		self.nu = 4

		# A discrete form 
		self.A = np.zeros((12,12))
		self.A[0:3,3:6] = np.eye(3)
		self.A[3,7] = self.g
		self.A[4,6] = -self.g
		self.A[6:9,9:12] = np.eye(3)
		self.A = np.eye(12) + self.ts*self.A

		# B discrete form
		self.B = np.zeros((12,4))
		self.B[5,:] = 1/m*np.array([1,1,1,1])
		self.B[9:12,:] = np.array([[l,-l,-l,l],[-l,-l,l,l],[k,-k,k,-k]])
		self.B[ 9,:] = self.B[ 9,:]/self.inertia_xx
		self.B[10,:] = self.B[10,:]/self.inertia_yy
		self.B[11,:] = self.B[11,:]/self.inertia_zz
		self.B = self.ts*self.B

		# Dynamics Affine Disturbance discrete form
		self.Bg= np.zeros((12,1))
		self.Bg[5] = -self.g
		self.Bg= self.ts*self.Bg.flatten()

		# Force Disturbance Parameter
		self.Fd = cvx.Parameter(3)
		self.Fd.value = np.zeros((3,1)).flatten()
		self.Bd = np.zeros((12,3))
		self.Bd[3,0] = self.ts/self.m 
		self.Bd[4,1] = self.ts/self.m
		self.Bd[5,2] = self.ts/self.m

		# Disturbance Observer 
		self.x_obs = np.zeros((15,1))
		self.C = np.eye(12)
		self.Cd = np.zeros((12,3))
		self.A_obs = np.block([[self.A,self.Bd],[np.zeros((3,12)),np.eye(3)]])
		self.B_obs = np.block([[self.B],[np.zeros((3,4))]])
		self.Bg_obs = np.block([[self.Bg.reshape((12,1))],[np.zeros((3,1))]])
		self.C_obs = np.block([[self.C,self.Cd]])
		poles = np.array([93.,94,95,96,97,98,99,100,101,102,103,104,105,106,107])/130
		sys = scipy.signal.place_poles(self.A_obs.transpose(),self.C_obs.transpose(),poles)
		self.L_obs = sys.gain_matrix.transpose()

		# Disturbance Rejection
		self.C = np.block([np.eye(3),np.zeros((3,9))])
		self.A_track = np.block([[self.A-np.eye(12),self.B],[self.C,np.zeros((3,4))]])

		# MPC cost function
		self.xbar = cvx.Parameter(12)
		self.xbar.value = xbar_v.flatten()
		self.ubar = cvx.Parameter(4)
		self.ubar.value = ubar_v.flatten()

		# Variable constraints
		xl = [None]*self.nx
		xu = [None]*self.nx
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
		self.X = cvx.Variable((self.nx,self.n + 1))
		self.U = cvx.Variable((self.nu,self.n))

		# Cost function
		self.J = cvx.quad_form(self.X[:,self.n]-self.xbar,self.P)
		#print(self.U[:,k].shape)
		#print(self.ubar.shape)
		#print(self.R.shape)
		#print('X',self.X[:,k].shape)
		#print('xbar',self.xbar)
		#print('Q',self.Q)
		#print(cvx.quad_form(self.X[:,k]-self.xbar,self.Q))
		#print('U',self.U[:,k].shape)
		#print('ubar',self.ubar)
		#print('U',self.U)
		#print(self.U[:,k].T*self.R*self.U[:,k])
		#print(cvx.quad_form(self.U[:,k]-self.ubar,self.R))
		#print('R',self.R)
		#print('I',np.eye(4))

		for k in range(self.n):
			self.J += cvx.quad_form(self.X[:,k]-self.xbar,self.Q) + cvx.quad_form(self.U[:,k]-self.ubar,np.eye(4))

		self.constraints = []
		# Dynamic Constraints		
		for k in range(0,self.n):
			self.constraints += [self.X[:,k+1] == self.A*self.X[:,k] + self.B*self.U[:,k] + self.Bg+ self.Bd*self.Fd]	

		# State Constraints
		self.constraints += [self.X[:,0] == self.x0] # Initial Constraint
		for k in range(1,self.n):
			for i in range(self.nx):
				if xl[i] is not None:
					self.constraints += [xl[i] <= self.X[i,k]]
				if xu[i] is not None:
					self.constraints += [self.X[i,k] <= xu[i]]

		# Input Constraints
		for k in range(0,self.n):
			for i in range(self.nu):
				if ul[i] is not None:
					self.constraints += [ul[i] <=self. U[i,k]]
				if uu[i] is not None:
					self.constraints += [self.U[i,k] <= uu[i]]

		self.problem = cvx.Problem(cvx.Minimize(self.J),self.constraints)

		self.counter = 1 # for when solver fails

	def actuate(self,x0):

		self.observe_Fd(x0)

		self.x0.value = x0.flatten()

		self.problem.solve()

		return self.U[:,0].value

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

		self.x_obs = np.dot(self.A_obs,self.x_obs) + \
					 np.dot(self.B_obs,self.U[:,0].value.reshape((4,1))) + \
					 self.Bg_obs - \
					 self.L_obs.dot(self.x_obs[0:12] - x.reshape((12,1)))

		dhat = self.x_obs[12:15]
		self.Fd.value = dhat.flatten()

		# solve for xtrack and utrack
		xtut = cvx.Variable((16,1)) # xt stacked on ut
		J = cvx.quad_form(xtut[12:16],np.eye(4))
		constraints = [self.A_track*xtut + np.block([[self.Bg.reshape((12,1))],[np.zeros((3,1))]]) == np.block([[-self.Bd.dot(dhat)],[self.r]])]
		problem = cvx.Problem(cvx.Minimize(J),constraints)
		problem.solve()

		xt = xtut[0:12].value
		ut = xtut[12:16].value

		self.change_xbar(xt)
		self.change_ubar(ut)

		self.Fd.value = self.x_obs[12:15].flatten()

		return
		


