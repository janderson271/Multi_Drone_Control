#!/usr/bin/env python
import numpy as np
import rospy
import rosnode
import tf
from geometry_msgs.msg import Pose, Twist, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def callback(msg, args):
	node, topic_dict = args
	print("hell")
	topic_dict[node] = msg

def main():
	rospy.init_node("tf_pub")
	nodes = []
	position_dict = {}
	tf_dict = {}
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		node_names = rosnode.get_node_names()
		print("A", node_names, position_dict)
		for node in node_names:
			node = node[1:]
			if node.startswith("drone") or node.startswith("box") and node not in nodes:
				nodes.append(node)
				rospy.Subscriber("/" + node + "/position", Pose, callback, (node, position_dict))
		for node in nodes:
			if node not in position_dict: continue
			curr_pose = position_dict[node]
			if node not in tf_dict:
				tf_dict[node] = tf.TransformBroadcaster()
			pos, ori = (curr_pose.position.x, curr_pose.position.y, curr_pose.position.z), \
				(curr_pose.orientation.x, curr_pose.orientation.y, curr_pose.orientation.z, curr_pose.orientation.w)
			print(node)
			tf_dict[node].sendTransform(pos, ori, rospy.Time.now(), node, "world")
		rate.sleep()

if __name__ == "__main__":
    try:
		main()
    except rospy.ROSInterruptException:
		pass
