#!/usr/bin/env python
import numpy as np
import rospy
import rosnode
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Twist, Vector3

def is_new_drone(topic, drone_topics):
	if topic[0:5] == "drone" and \
	    topic[0:6] not in drone_topics:
		return topic[0:6]
	return False

def callback(msg, args):
	drone, topic_dict = args
	topic_dict[drone] = msg

def make_pub(drone):
	return rospy.Publisher("viz" + drone + "/marker", Marker, queue_size=1)

def drone_viz():
	rospy.init_node("viz")
	drones = []
	position_dict = {}
	pub_dict = {}
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		#on fixed timestip: simulate the system and publish
		nodes = rosnode.get_node_names()
		for node in nodes:
			node = node[1:]
			if node.startswith("drone") and node not in drones:
				drones.append(node)
				rospy.Subscriber("/" + node + "/position", Pose, callback, (node, position_dict))
		for drone in drones:
			if drone not in position_dict: continue
			marker = Marker()
			marker.header.frame_id = "world"
			marker.header.stamp = rospy.Time.now()
			marker.type = Marker.CYLINDER
			marker.action = Marker.ADD
			marker.pose = position_dict[drone]
			marker.scale = Vector3(1,1,0.1)
			marker.color.r = 0.0
			marker.color.g = 1.0
			marker.color.b = 0.0
			marker.color.a = 1.0
			marker.lifetime = rospy.Duration()
			if drone not in pub_dict:
				pub_dict[drone] = make_pub(drone)
			pub = pub_dict[drone]
			pub.publish(marker)
		rate.sleep()

if __name__ == "__main__":
    try:
		drone_viz()
    except rospy.ROSInterruptException:
		pass
