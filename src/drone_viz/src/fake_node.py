#!/usr/bin/env python
import rospy

if __name__ == '__main__':
	try:
	 	rospy.init_node('node')
	except rospy.ROSInterruptException:
		pass