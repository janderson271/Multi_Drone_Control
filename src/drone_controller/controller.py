#!/usr/bin/env python
import numpy as np
import cvxpy as cvx
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import rospy
from geometry_msgs.msg import Pose, Twist, Wrench
from std_msgs.msg import Float32MultiArray

def actuate():

	rospy.init_node("controller")
	drone_name = rospy.get_param("drone")
	node_name = rospy.get_name()

	control_pub = rospy.Publisher(drone_name + "/control", Float32MultiArray, queue_size = 1)
	controller = Controller()
	pos_sub = rospy.Subscriber(drone_name + "/position", Pose, controller.pos_callback)
	vel_sub = rospy.Subscriber(drone_name + "/velocity", Twist, controller.vel_callback)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		uOpt = controller.calc_opt_actuation()
		control_pub.publish(uOpt)
		rate.sleep()

class Controller: 

	def __init__(self, drone,ts = 0.1, x0=np.zeros((12,1))):

		self.drone = drone

		self.ts = ts

		# drone characteristics
		l = drone.length
		w = drone.width
		m = drone.mass
		J = drone.inertia
		self.x0 = x0

		# drone dynamics
		self.nx = 12
		A = np.zeros((12,12))
		A[0:3,3:6] = np.eye(3)
		A[3,7] = g
		A[4,6] = -g
		A[6:9,9:12] = np.eye(3)
		self.A = np.eye(12) + ts*A # ZOH discretization

		self.nu = 4
		k = 1
		B[5,:] = 1/m*np.array([1,1,1,1])
		B[9:12,:] = np.array([[l,-l,-l,l],[-l,-l,l,l],[-k,k,-k,k]])
		self.B = ts*B # ZOH discretization

		Bd = zeros((12,1))
		Bd[5] = -g
		self.Bd = ts*Bd # ZOH discretization

		# Quadratic Cost Function
		self.xbar = np.zeros((self.nx, 1))
		self.P    = np.zeros((self.nx,self.nx))
		self.Q    = np.zeros((self.nx,self.nx))
		self.ubar = np.zeros((self.nu,1))
		self.R    = np.zeros((self.nu,self.nu))
		
		# Horizon
		self.n = 10

		# state constraints
		self.xL = np.zeros((self.nx,1))
		self.xU = np.zeros((self.nx,1))
		self.uL = np.zeros((self.nu,1))
		self.uU = np.zeros((self.nx,1))

		# terminal constraints
		self.bf = np.zeros((self.nx,1))
		self.Af = np.identity(self.nx)

	def calc_opt_actuation(self):
		return solve_cftoc_CVXPY(self)

	def solve_cftoc_YALMIP(self):

		eng = matlab.engine.start_matlab()
		eng.addpath(r'~/Documents/MATLAB/YALMIP-master',nargout=0)
		eng.addpath(r'~/Documents/MATLAB/YALMIP-master/demos',nargout=0)
		eng.addpath(r'~/Documents/MATLAB/YALMIP-master/extras',nargout=0)
		eng.addpath(r'~/Documents/MATLAB/YALMIP-master/modules',nargout=0)
		eng.addpath(r'~/Documents/MATLAB/YALMIP-master/operators',nargout=0)
		eng.addpath(r'~/Documents/MATLAB/YALMIP-master/@sdpvar',nargout=0)
		eng.addpath(r'~/Documents/MATLAB/YALMIP-master/solvers',nargout=0)

		return eng.solve_cftoc_YALMIP( matlab.double(self.A.tolist()), \
							   matlab.double(self.B.tolist()), \
							   matlab.double(self.Bd.tolist()), \
							   matlab.double(self.P.tolist()), \
							   matlab.double(self.Q.tolist()), \
							   matlab.double(self.R.tolist()), \
 							   matlab.double(self.n.tolist()), \
 							   matlab.double(self.xbar.tolist()), \
 							   matlab.double(self.ubar.tolist()), \
 							   matlab.double(self.x0.tolist()), \
 							   matlab.double(self.xL.tolist()), \
 							   matlab.double(self.xU.tolist()), \
 							   matlab.double(self.uL.tolist()), \
 							   matlab.double(self.uU.tolist()), \
 							   matlab.double(self.bf.tolist()), \
 							   matlab.double(self.Af.tolist()))

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

	def pos_callback(self,pos_msg):
		self.x0 = np.array(pos_msg.data)

	def vel_callback(self,vel_msg):
		self.x0 = np.array(vel_msg.data)

	def external_callback(self,external_msg):
		self.Fext = self.vectornp(external_msg.force)

        def vectornp(self, msg): return np.array([msg.x, msg.y, msg.z]) 

if __name__ == "__main__":
    try:
		sim()
    except rospy.ROSInterruptException:
		pass
