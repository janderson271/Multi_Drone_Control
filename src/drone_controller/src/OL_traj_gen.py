import numpy as np
import cvxpy as cvx

class OL_traj_gen():

	def __init__(self):
			
	def solve_cftoc(self, xf_v = np.zeros((12,1)), 
							 ul=np.zeros((4,1)).flatten(), 
							 uu=100*np.ones( 4).flatten(), 
							 n=100,
							 ts = 0.1,
							 P = np.ones((12,1)), 
							 Q = np.ones((12,1)), 
							 R = np.ones(( 4,1)),
					         x0_v = np.zeros((12, 1)),
					         xl = np.array([-10., -10., 0, -10., -10., -10., -25*np.pi/180, -25*np.pi/180, -25*np.pi/180, -.05, -.05, -.05]),
					         xu = np.array([10., 10., 10., 10., 10., 10., 25*np.pi/180, 25*np.pi/180, 25*np.pi/180, .05, .05, .05])):

		P = np.diag(P)
		Q = np.diag(Q)
		R = np.diag(R)

		l=0.033
		m=0.032
		inertia_xx = 16e-6
		inertia_yy = inertia_xx
		inertia_zz = 29e-6
		k=0.01
		
		g = 9.81

		x0 = cvx.Parameter(12)
		x0.value = x0_v.flatten()


		xf = cvx.Parameter(12)
		xf.value = xf_v.flatten()

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
	
		# Variable constraints
		#xl = [None]*nx
		#xu = [None]*nx
		#d = 25
		#for i in range(6,8):
		#xl = -d*3.14/180
		#xu =  d*3.14/180

		#ul =  np.zeros((4,1)).flatten()
		#uu =  5*np.ones( 4).flatten()

		# CVXPY Formulation
		X = cvx.Variable((nx,n + 1))
		U = cvx.Variable((nu,n))
		
		# Cost function
		J = cvx.quad_form(X[:,n]-xf,P)
		for k in range(n):
			J += cvx.quad_form(X[:,k]-xf,Q) + cvx.quad_form(U[:,k],R)

		constraints = []
		# Dynamic Constraints		
		for k in range(0,n):
			constraints += [X[:,k+1] == A*X[:,k] + B*U[:,k] + Bd]	

		# State Constraints
		constraints += [X[:,0] == x0] # Initial Constraint
		for k in range(0,n+1):
			for i in range(nx):
				constraints += [xl[i] <= X[i,k]]
				constraints += [X[i,k] <= xu[i]]

		# Input Constraints
		for k in range(n):
			for i in range(nu):
				if ul[i] is not None:
					constraints += [ul[i] <= U[i,k]]
				if uu[i] is not None:
					constraints += [U[i,k] <= uu[i]]

		problem = cvx.Problem(cvx.Minimize(J),constraints)
		problem.solve(solver=cvx.CVXOPT)
		if problem.status == "optimal":
			return  X.value, U.value

		return problem
