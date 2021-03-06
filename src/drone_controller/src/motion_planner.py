#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray, Int32
import ipdb
from OL_traj_gen import OL_traj_gen
from tf.transformations import euler_from_quaternion, quaternion_from_euler

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

def set_pub_sub(num_drones): 
	subscribers = []
	for i in range(1, num_drones + 1):
		tracker = Tracker("drone_" + str(i))
		subscribers.append(tracker)
	return subscribers

def motion_planner():
	rospy.init_node("god")
	node_name = rospy.get_name()
	num_drones = rospy.get_param(node_name + "/num_drones")

	timer = Timer()
	time_sub = rospy.Subscriber("god/time", Int32, timer.timer_callback)

	# CORNERS OF A SQUARE
	# # 4 waypoints
	# nominal_traj = np.zeros((12,4))

	# # all at z = 2
	# nominal_traj[2,:] = 2.
	# nominal_traj[2,2] = 3.
	
	# # waypoint 1
	# nominal_traj[0,0] = 0.
	# nominal_traj[1,0] = 0.

	# # waypoint 2
	# nominal_traj[0,1] = 2.
	# nominal_traj[1,1] = 0.

	# # waypoint 3
	# nominal_traj[0,2] = 2.
	# nominal_traj[1,2] = 2.


	# # waypoint 4
	# nominal_traj[0,3] = 0.
	# nominal_traj[1,3] = 2.

	nominal_traj = np.zeros((12,5))

	nominal_traj[2,:] = 2.5

	nominal_traj[0,0] = 0.
	nominal_traj[1,0] = 0.

	nominal_traj[0,1] = 1.06
	nominal_traj[1,1] = 0.44

	nominal_traj[0,2] = 1.5
	nominal_traj[1,2] = 1.5

	nominal_traj[0,3] = 1.94
	nominal_traj[1,3] = 0.44

	nominal_traj[0,4] = 0.
	nominal_traj[1,4] = 3.

	drones = set_pub_sub(num_drones)
	for drone in drones:
		drone.Xref = np.copy(nominal_traj)
		drone.x0 = rospy.get_param("{}/{}".format(drone.name, "x0"))
		for i in range(len(nominal_traj[0])-1):
			if i == 0:
				t = 0
			elif (nominal_traj[0,i+1] - nominal_traj[0,i]) == 0:
				t = 0
			else:
				t = np.arctan((nominal_traj[1,i+1] - nominal_traj[1,i])/(nominal_traj[0,i+1] - nominal_traj[0,i]))
			# t = 0
			R = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
			xnew = R.dot(np.array([[drone.x0[0]], [drone.x0[1]]]))
			drone.Xref[0,i] += xnew[0][0]
			drone.Xref[1,i] += xnew[1][0]

	while not rospy.is_shutdown():
		if timer.global_time >= timer.time:
			timer.time += 1
			all_close = sum(1 for drone in drones if np.linalg.norm(drone.xbar[:3] - drone.x[:3]) < drone.thresh) == num_drones
			for drone in drones:
				drone.pub_next_waypoint(all_close)

class Timer:
	def __init__(self):
		self.time = 0
		self.global_time = 0

	def timer_callback(self, time_msg):
		self.global_time = time_msg.data
		
class Tracker: 
	def __init__(self, name):
		self.max_reached = 0
		self.finished = False
		self.msg = Pose()
		self.x0 = np.zeros(12)
		self.x = np.zeros(12)
		self.name = name
		self.pub = rospy.Publisher(name + "/waypoint", Float32MultiArray, queue_size=10)
		self.sub = rospy.Subscriber(name + "/position", Pose, self.state_callback)
		self.Xref = np.zeros((12,1))
		self.Xref[0:3] = np.ones((3,1))
		self.xbar = self.Xref[:,0]
		self.thresh = 0.1

	def pub_next_waypoint(self, all_close):
		if all_close and self.max_reached + 1 != self.Xref.shape[1]: 
			self.max_reached += 1
		self.xbar = self.Xref[:, self.max_reached]
		msg = Float32MultiArray()
		msg.data = self.xbar
		self.pub.publish(msg)

	def state_callback(self, pos_msg):
		self.msg = pos_msg
		self.x[0:3] = np.array([pos_msg.position.x, pos_msg.position.y, pos_msg.position.z])
		self.x[6:9] = np.array(euler_from_quaternion([pos_msg.orientation.x, \
									   		     pos_msg.orientation.y, \
												 pos_msg.orientation.z, \
												 pos_msg.orientation.w]))

    
if __name__ == "__main__":
    try:
	 	motion_planner()
    except rospy.ROSInterruptException:
		pass
