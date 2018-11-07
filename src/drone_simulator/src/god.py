#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def stuff():
	rospy.init_node("node_name")
	pub = rospy.Publisher("topic_name", Twist, queue_size=1)
	
	while not rospy.is_shutdown():
		msg = Twist()
		msg.linear.x = 2
		pub.publish(msg)

class God: 
	def __init__(self):
		pass

    
if __name__ == "__main__":
    try:
	 	stuff()
    except rospy.ROSInterruptException:
		pass
