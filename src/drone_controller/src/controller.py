#!/usr/bin/env python
import numpy as np
import cvxpy as cvx
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import rospy
from geometry_msgs.msg import Pose, Twist, Wrench
from std_msgs.msg import Float32MultiArray, Int32

import MPCcontroller as MPC

def control():
	rospy.init_node("controller")
	node_name = rospy.get_name()
	drone_name = rospy.get_param(node_name + "/drone")

	droneController = DroneController(MPC.MPCcontroller())
	#droneController = DroneController(LQR.LQRcontroller())

	control_pub = rospy.Publisher(drone_name + "/control", Float32MultiArray, queue_size = 1)
	pos_sub = rospy.Subscriber(drone_name + "/position", Pose, droneController.pos_callback)
	vel_sub = rospy.Subscriber(drone_name + "/velocity", Twist, droneController.vel_callback)
	time_sub = rospy.Subscriber("god/time", Int32, droneController.timer_callback)
	rate = rospy.Rate(10)
	control_input = Float32MultiArray()

	
	while not rospy.is_shutdown():
		if droneController.time == droneController.global_time:
			uOpt = droneController.calc_actuation()
			if uOpt is not None:
				control_input.data = uOpt
				control_pub.publish(control_input)
		rate.sleep()

class DroneController:
	def __init__(self, controller = None):
		self.x = np.zeros(12)
		self.controller = controller
		self.time = 0
		self.global_time = 0

	def calc_actuation(self):
		self.time += 1
		return self.controller.actuate(self.x.flatten())

	def pos_callback(self,pos_msg):
		self.x[0:3] = np.array([pos_msg.position.x, pos_msg.position.y, pos_msg.position.z])
		self.x[6:9] = np.array(euler_from_quaternion([pos_msg.orientation.x, \
									   		     pos_msg.orientation.y, \
												 pos_msg.orientation.z, \
												 pos_msg.orientation.w]))

	def vel_callback(self,vel_msg):
		self.x[3:6] = np.array([vel_msg.linear.x, vel_msg.linear.y, vel_msg.linear.z])
		self.x[9:12]= np.array([vel_msg.angular.x, vel_msg.angular.y, vel_msg.angular.z])

	def external_callback(self,external_msg):
		self.Fext = self.vectornp(external_msg.force)
	def vectornp(self, msg): return np.array([msg.x, msg.y, msg.z]) 

	def timer_callback(self, time_msg):
		self.global_time = time_msg.data

if __name__ == "__main__":
    try:
		control()
    except rospy.ROSInterruptException:
		pass
