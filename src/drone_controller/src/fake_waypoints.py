#!/usr/bin/env python
import numpy as np
import rospy
import rosnode
import ipdb
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose, Wrench, Vector3

def sim():
	node_name = 'fake_waypoints'
	rospy.init_node(node_name)

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		drone_node_names = rosnode.get_node_names()
		for drone_node in drone_node_names:
			if drone_node.startswith('/drone'):
				goal = Float32MultiArray
				goal.data = np.array([1,1,1])
				waypoint_pub = rospy.Publisher(drone_node + '/waypoint', Float32MultiArray, queue_size=10)
				waypoint_pub.publish(goal)
		rate.sleep()
		
if __name__ == '__main__':
    try:
		sim()
    except rospy.ROSInterruptException:
		pass
