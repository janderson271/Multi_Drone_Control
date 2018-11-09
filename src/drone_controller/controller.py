#!/usr/bin/env python
import numpy as np
import cvxpy as cvx
import rospy
from geometry_msgs.msg import Pose, Twist, Wrench
from std_msgs.msg import Float32MultiArray

def sim():
	rospy.init_node("controller")
	node_name = rospy.get_name()

	# pos_pub = rospy.Publisher(node_name + "/position", Pose, queue_size=1)
	# vel_pub = rospy.Publisher(node_name + "/velocity", Twist, queue_size=1)
	# drone = Drone()
	# control_sub = rospy.Subscriber(node_name + "/control", Float32MultiArray, drone.control_callback)
	# control_sub = rospy.Subscriber(node_name + "/external_force", Wrench, drone.control_callback)
	# rate = rospy.Rate(1/drone.dt)

	control_pub = rospy.Publisher(node_name + "/control", Float32MultiArray, queue_size = 1)
	controller = controller()
	pos_sub = rospy.Subscriber(node_name + "/state", Pose, controller.state_callback)
	rate = rospy.Rate(1/controller.dt)

	while not rospy.is_shutdown():
		#on fixed timestip: simulate the system and publish
		
		# position, velocity = drone.sim_step()
		# pos_pub.publish(position)
		# vel_pub.publish(velocity)

		uOpt = controller.calc_opt_actuation()
		control_pub.publish()
		rate.sleep()


class Controller: 

	def __init__(self, drone,ts = 0.1, x0=None):

		self.ts = ts

		# drone characteristics
		l = drone.length
		w = drone.width
		m = drone.mass
		J = drone.inertia

		# drone dynamics
		self.nx = 12
		A = np.zeros((nx,nx))
		A[0:3,3:6] = np.eye(3)
		A[3,7] = g
		A[4,6] = -g
		A[6:9,9:12] = np.eye(3)
		self.A = ts*A
		#

		self.nu = 4
		k = 1
		B[5,:] = 1/m*np.array([1,1,1,1])
		B[9:12,:] = np.array([[l,-l,-l,l],[-l,-l,l,l],[-k,k,-k,k]])
		self.B = B
		# MUST ADD GRAVITATION TO DYNAMICS CONSTRAINTS

		self.x0 = x0

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

		return None

	def solve_cftoc(self):

		return None

		X = cvx.Variable((self.nx,self.n + 1))
		U = cvx.Variable((self.nu,self.n))

		J = cvx.Minimize(               X[:,self.n].T*self.P*X[:,self.n]       + \
					     cvx.trace( X[:,0:self.n-1].T*self.Q*X[:,0:self.n-1] ) + \
					     cvx.trace(				  U.T*self.R*U               )     ) 

		constraints = []

		# Dynamic Constraintss
		constraints += [X[:,0] == self.x0]

		#for k in range(0,self.n-1):
			constraints += [X[:,k+1] == self.A*X[:,k] + self.B*U[:,k] ]

		# State Constraints
		for k in range(1,self.n):
			constraints += [self.xL <= X[:,k], X[:,k] <= self.xU]

		# Input Constraints
		for k in range(0,self.n-1):
			constraints += [self.uL <= U[:,k], U[:,k] <= self.uU]

		# Terminal Constraint
		constraints += [self.Af*X[:,self.n] <= self.bf]

		# Solve Program
		problem = cvx.Problem(J,constraints)
		solution = problem.solve()

		return U[:,0]


	# def sim_step(self):
		"""
		self.u = self.nux1 np array [FL, FR, RL, RR]
		self.F_ext = 3x1 np array [x component, y component, z component]
		update self.x (self.nxx1 state vector)
		use euler angles
		sets the internal state and returns pose and twist for the drone
		""" 
		#TODO: set self.x = f(x,u,F_ext)
		#return Pose(), Twist()

	def state_callback(self,state_msg):
		self.x0 = np.array(state_msg.data)

	def external_callback(self,external_msg):
		self.Fext = self.vectornp(external_msg.force)

        def vectornp(self, msg): return np.array([msg.x, msg.y, msg.z]) 

if __name__ == "__main__":
    try:
		sim()
    except rospy.ROSInterruptException:
		pass
