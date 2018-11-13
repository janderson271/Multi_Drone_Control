#!/usr/bin/env python
import numpy as np
import cvxpy as cvx
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import rospy
from geometry_msgs.msg import Pose, Twist, Wrench
from std_msgs.msg import Float32MultiArray

def actuate():

	rospy.init_node("controller")
	node_name = rospy.get_name()
	drone_name = rospy.get_param(node_name + "/drone")

	control_pub = rospy.Publisher(drone_name + "/control", Float32MultiArray, queue_size = 1)
	controller = Controller()
	pos_sub = rospy.Subscriber(drone_name + "/position", Pose, controller.pos_callback)
	vel_sub = rospy.Subscriber(drone_name + "/velocity", Twist, controller.vel_callback)
	rate = rospy.Rate(10)
	control_input = Float32MultiArray()
	while not rospy.is_shutdown():
		uOpt = controller.calc_opt_actuation()
		control_input.data = uOpt
		control_pub.publish(uOpt)
		rate.sleep()

class Controller: 

	def __init__(self, ts = 0.1, x0=np.zeros((12,1)), length=0.5, width=0.5, mass=1, inertia_xx=.45, inertia_yy =0.45, inertia_zz=0.7, k=0.3):

		self.ts = ts

		# drone characteristics
		l = length
		w = width
		m = mass
		J = np.diag([inertia_xx, inertia_yy, inertia_zz])

		# drone dynamics
		self.x0 = x0
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
		self.xbar = np.zeros((self.nx, 1)).flatten()
		self.xbar[2] = 1
		self.P    = np.zeros((self.nx,self.nx))
		self.Q    = np.eye(self.nx)
		self.ubar = np.zeros((self.nu,1)).flatten()
		self.R    = np.eye(self.nu)
		
		# Horizon
		self.n = 10

		# state constraints
		m = 100
		self.xL = -m*np.ones(12).flatten()
		self.xU =  m*np.ones(12).flatten()
		self.uL =  np.zeros((4,1)).flatten()
		self.uU =  m*np.ones( 4).flatten()

		# terminal constraints
		# Ax==b
		xf = np.zeros((12,1))
		xf[2] = 1
		self.bf = np.vstack((xf,-xf)).flatten()
		self.Af = np.vstack((np.eye(12),-np.eye(12)))

	def calc_opt_actuation(self):
		return self.solve_cftoc_CVXPY()

	def solve_cftoc_CVXPY(self):

		X = cvx.Variable((self.nx,self.n + 1))
		U = cvx.Variable((self.nu,self.n))

		# Cost function
		J = cvx.quad_form(X[:,self.n],self.P)
		for k in range(self.n):
			J += cvx.quad_form(X[:,k]-self.xbar,self.Q) + cvx.quad_form(U[:,k]-self.ubar,self.R)

		constraints = []
		# Dynamic Constraints
		import ipdb; ipdb.set_trace()		
		for k in range(0,self.n):
			constraints += [X[:,k+1] == self.A*X[:,k] + self.B*U[:,k] + self.Bd]	

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

	def pos_callback(self,pos_msg):
		self.x0[0:3] = np.array([pos_msg.position.x, pos_msg.position.y, pos_msg.position.z]).reshape(3,1)
		self.x0[6:9] = np.array(euler_from_quaternion([pos_msg.orientation.x, pos_msg.orientation.y, pos_msg.orientation.z, pos_msg.orientation.w])).reshape(3,1)

	def vel_callback(self,vel_msg):
		self.x0[3:6] = np.array([vel_msg.linear.x, vel_msg.linear.y, vel_msg.linear.z]).reshape(3,1)
		self.x0[9:] = np.array([vel_msg.angular.x, vel_msg.angular.y, vel_msg.angular.z]).reshape(3,1)

	def external_callback(self,external_msg):
		self.Fext = self.vectornp(external_msg.force)

        def vectornp(self, msg): return np.array([msg.x, msg.y, msg.z]) 

if __name__ == "__main__":
    try:
		actuate()
    except rospy.ROSInterruptException:
		pass
