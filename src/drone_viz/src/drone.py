#!/usr/bin/env python
import numpy as np
import rospy
import rosnode
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Twist, Vector3

def callback(msg, args):
	drone, topic_dict = args
	topic_dict[drone] = msg

def drone_viz():
	rospy.init_node("viz")
	drones = []
	position_dict = {}
	pub_dict = {}
	rate = rospy.Rate(10)
	pub = rospy.Publisher("viz/drone_markers", MarkerArray, queue_size=1)
	while not rospy.is_shutdown():
		#on fixed timestip: simulate the system and publish
		marker_array = MarkerArray()
		nodes = rosnode.get_node_names()
		for node in nodes:
			node = node[1:]
			if node.startswith("drone") and node not in drones:
				drones.append(node)
				rospy.Subscriber("/" + node + "/position", Pose, callback, (node, position_dict))
		t, lifetime = rospy.Time.now(), rospy.Duration()
		for i, drone in enumerate(drones):
			if drone not in position_dict: continue
			marker = Marker()
			marker.header.frame_id = "world"
			marker.id = i
			marker.header.stamp = t
			marker.type = Marker.MESH_RESOURCE
			marker.mesh_resource = "package://drone_viz/src/drone.obj"
			marker.action = Marker.ADD
			marker.pose = position_dict[drone]
			marker.pose.orientation.x += np.pi / 3
			marker.scale = Vector3(0.1,0.1,0.1)
			marker.color.r = 0.0
			marker.color.g = 1.0
			marker.color.b = 0.0
			marker.color.a = 1.0
			marker.lifetime = lifetime
			marker_array.markers.append(marker)
		pub.publish(marker_array)
		rate.sleep()

if __name__ == "__main__":
    try:
		drone_viz()
    except rospy.ROSInterruptException:
		pass
