#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int32

def set_subscribers(num_drones): 
	subscribers = []
	for i in range(1, num_drones + 1):
		tracker = Tracker()
		sub = rospy.Subscriber("drone_" + str(i) + "/control", Float32MultiArray, tracker.actuation_callback)
		subscribers.append(tracker)
	return subscribers

def watch():
	rospy.init_node("god")
	node_name = rospy.get_name()
	num_drones = rospy.get_param(node_name + "/num_drones")

	pub = rospy.Publisher(node_name + "/time", Int32, queue_size=0)
	subscribers = set_subscribers(num_drones)

	time = Int32()
	n = 0
	ready = True
	while not rospy.is_shutdown():
		for sub in subscribers:
			if sub.control_counter != n + 1:
				ready = False
		if ready:
			n += 1
			time.data = n
			print('ready!')
			pub.publish(time)
		else :
			ready = True
		

class Tracker: 
	def __init__(self):
		self.control_counter = 0
		self.msg = Float32MultiArray()

	def actuation_callback(self, msg):
		self.control_counter += 1
		self.msg = msg

    
if __name__ == "__main__":
    try:
	 	watch()
    except rospy.ROSInterruptException:
		pass
