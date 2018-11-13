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
		#print('uOpt = ' + str(uOpt))
		control_input.data = uOpt
		control_pub.publish(control_input)
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
		self.x0 = cvx.Parameter(12)
		self.x0.value = x0.flatten()
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
		self.Q    = 10*np.eye(self.nx)
		self.ubar = np.zeros((self.nu,1)).flatten()
		self.R    = np.eye(self.nu)
		
		# Horizon
		self.n = 10

		# state constraints
		m = 100
		self.xL = -m*np.ones(12).flatten()
		self.xU =  m*np.ones(12).flatten()
		self.uL =  np.zeros((4,1)).flatten()
		self.uU =  5*np.ones( 4).flatten()

		# terminal constraints
		# Ax==b
		xf = np.zeros((12,1))
		xf[2] = 1
		self.bf = np.vstack((xf,-xf)).flatten()
		self.Af = np.vstack((np.eye(12),-np.eye(12)))

		self.cftoc_solver_CVXPY()

	def calc_opt_actuation(self):
		return self.solve_cftoc_CVXPY()

	def cftoc_solver_CVXPY(self):

		self.X = cvx.Variable((self.nx,self.n + 1))
		self.U = cvx.Variable((self.nu,self.n))

		# Cost function
		self.J = cvx.quad_form(self.X[:,self.n],self.P)
		for k in range(self.n):
			self.J += cvx.quad_form(self.X[:,k]-self.xbar,self.Q) + cvx.quad_form(self.U[:,k]-self.ubar,self.R)

		self.constraints = []
		# Dynamic Constraints		
		for k in range(0,self.n):
			self.constraints += [self.X[:,k+1] == self.A*self.X[:,k] + self.B*self.U[:,k] + self.Bd]	

		# State Constraints
		self.constraints += [self.X[:,0] == self.x0] # Initial Constraint
		for k in range(1,self.n):
			self.constraints += [self.xL <= self.X[:,k], self.X[:,k] <= self.xU]
		self.constraints += [self.Af*self.X[:,self.n] <= self.bf] # Terminal Constraint

		# Input Constraints
		for k in range(0,self.n):
			self.constraints += [self.uL <= self.U[:,k], self.U[:,k] <= self.uU]

		# Problem
		self.problem = cvx.Problem(cvx.Minimize(self.J),self.constraints)

		return 

	def solve_cftoc_CVXPY(self):
		self.problem.solve(warm_start=True)
		return self.U[:,0].value

	def pos_callback(self,pos_msg):
		self.x0.value[0:3] = np.array([pos_msg.position.x, pos_msg.position.y, pos_msg.position.z]).reshape(3,)
		self.x0.value[6:9] = np.array(euler_from_quaternion([pos_msg.orientation.x, pos_msg.orientation.y, pos_msg.orientation.z, pos_msg.orientation.w])).reshape(3,)

	def vel_callback(self,vel_msg):
		self.x0.value[3:6] = np.array([vel_msg.linear.x, vel_msg.linear.y, vel_msg.linear.z]).reshape(3,)
		self.x0.value[9:12]= np.array([vel_msg.angular.x, vel_msg.angular.y, vel_msg.angular.z]).reshape(3,)

	def external_callback(self,external_msg):
		self.Fext = self.vectornp(external_msg.force)

        def vectornp(self, msg): return np.array([msg.x, msg.y, msg.z]) 

if __name__ == "__main__":
    try:
		actuate()
    except rospy.ROSInterruptException:
		pass
