#!/usr/bin/env python
import numpy as np
import cvxpy as cvx
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import rospy
from geometry_msgs.msg import Pose, Twist, Wrench
from std_msgs.msg import Float32MultiArray, Int32
import matplotlib.pyplot as plt

import MPClifter as MPC

def get_and_set_params(node_name, params):
	# params is a dict from param_name -> default_value, None if no default value
	vals = {}
	for param, default in params.items():
		if default is None:
			vals[param] = rospy.get_param("{}/{}".format(node_name, param))
		else:
			if not rospy.has_param("{}/{}".format(node_name, param)):
				rospy.set_param("{}/{}".format(node_name, param), default)
			vals[param] = rospy.get_param("{}/{}".format(node_name, param))
	return vals

def control():
	rospy.init_node("controller")
	node_name = rospy.get_name()
	drone_name = rospy.get_param(node_name + "/drone")

	params_dict = dict(ts=None, P=None, Q=None, R=None, xbar=None, ubar=None, x0=None)
	params = get_and_set_params(node_name, params_dict)
	droneController = DroneController(MPC.MPCcontroller(np.array(params['ts'])  ,\
														np.array(params['P'])   ,\
														np.array(params['Q'])   ,\
														np.array(params['R'])   ,\
														np.array(params['xbar']),\
														np.array(params['ubar']),\
														np.array(params['x0'])     ))

	control_pub = rospy.Publisher(drone_name + "/control", Float32MultiArray, queue_size = 1)
	waypoint_sub = rospy.Subscriber(drone_name + "/waypoint", Float32MultiArray, droneController.waypoint_callback)
	pos_sub = rospy.Subscriber(drone_name + "/position", Pose, droneController.pos_callback)
	vel_sub = rospy.Subscriber(drone_name + "/velocity", Twist, droneController.vel_callback)
	time_sub = rospy.Subscriber("god/time", Int32, droneController.timer_callback)
	rate = rospy.Rate(10)
	control_input = Float32MultiArray()

	# plt.figure()
	
	while not rospy.is_shutdown():
		if droneController.time == droneController.global_time:
			uOpt = droneController.calc_actuation()
			if uOpt is not None:
				control_input.data = uOpt
				control_pub.publish(control_input)
				# droneController.plot_things()
				# plt.pause(.01)
		rate.sleep()

class DroneController:
	def __init__(self, controller = None):
		self.x = np.zeros(12)
		self.controller = controller
		self.time = 0
		self.global_time = 0

	def plot_things(self):
		X = self.controller.X.value
		x, y, z = X[0, :], X[1, :], X[2, :]
		plt.subplot(3,1,1)
		plt.cla()
		plt.plot(x, 'ro-')
		plt.plot(self.x[0], 0, 'k')
		plt.ylim([0, 10])
		plt.subplot(3,1,2)
		plt.cla()
		plt.plot(y, 'ro-')
		plt.plot(self.x[1], 0, 'k')
		plt.ylim([0, 10])
		plt.subplot(3,1,3)
		plt.cla()
		plt.plot(z, 'ro-')
		plt.plot(self.x[2], 0, 'k')
		plt.ylim([0, 10])
		

	def calc_actuation(self):
		self.time += 1
		return self.controller.actuate(self.x.flatten())

	def pos_callback(self,pos_msg):
		self.x[0:3] = np.array([pos_msg.position.x, pos_msg.position.y, pos_msg.position.z])
		self.x[6:9] = np.array(euler_from_quaternion([
													pos_msg.orientation.x, 
									   		        pos_msg.orientation.y, 
												    pos_msg.orientation.z, 
												    pos_msg.orientation.w
												    ]))

	def vel_callback(self,vel_msg):
		self.x[3:6] = np.array([vel_msg.linear.x, vel_msg.linear.y, vel_msg.linear.z])
		self.x[9:12]= np.array([vel_msg.angular.x, vel_msg.angular.y, vel_msg.angular.z])

	def external_callback(self,external_msg):
		self.Fext = self.vectornp(external_msg.force)

	def waypoint_callback(self, waypoint_msg):
		self.controller.r = np.array(waypoint_msg.data[0:3])

	def vectornp(self, msg): return np.array([msg.x, msg.y, msg.z]) 

	def timer_callback(self, time_msg):
		self.global_time = time_msg.data

if __name__ == "__main__":
    try:
		control()
    except rospy.ROSInterruptException:
		pass
