#!/usr/bin/env python

import rospy
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

	drones = set_pub_sub(num_drones)
	for drone in drones:
		drone.x0 = rospy.get_param("{}/{}".format(drone.name, "x0"))

	while not rospy.is_shutdown():
		if timer.global_time >= timer.time:
			timer.time += 1
			for drone in drones:
				drone.pub_next_waypoint()

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

	def pub_next_waypoint(self):
		xbar = self.Xref[:, self.max_index]
		curr_closeness = np.linalg.norm(xbar[:3] - self.x[:3])
		thresh = 0.1
		if curr_closeness < thresh and self.max_index + 1 != self.Xref.shape[1]: 
			self.max_index += 1
		xbar = self.Xref[:, self.max_index]
		msg = Float32MultiArray()
		msg.data = xbar[0:6]
		self.pub(msg)

	def state_callback(self, msg):
		self.msg = msg
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
