#!/usr/bin/env python
import numpy as np
import rospy
import rosnode
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Twist, Vector3, Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float32MultiArray

def callback(msg, args):
	drone, topic_dict = args
	topic_dict[drone] = msg

def drone_viz():
	rospy.init_node("viz")
	drones = []
	position_dict = {}
	waypoint_dict = {}
	pub_dict = {}
	rate = rospy.Rate(10)
	pub = rospy.Publisher("viz/drone_markers", MarkerArray, queue_size=1)
	markers = {}
	waypoints = {}
	trajectories = {}
	marker_array = MarkerArray()
	global_id = 0
	BOX = "box"
	while not rospy.is_shutdown():
		#on fixed timestip: simulate the system and publish
		nodes = rosnode.get_node_names()
		for node in nodes:
			node = node[1:]
			if node.startswith("drone") and node not in drones or node == BOX:
				drones.append(node)
				rospy.Subscriber("/" + node + "/position", Pose, callback, (node, position_dict))
				rospy.Subscriber("/" + node + "/waypoint", Float32MultiArray, callback, (node, waypoint_dict))
		t, lifetime = rospy.Time.now(), rospy.Duration()
		if BOX in drones and BOX in position_dict:
			if BOX not in markers:
				markers[BOX] = Marker()
				marker = markers[BOX]
				marker.header.frame_id = "world"
				marker.id = global_id
				global_id += 1
				marker.header.stamp = t
				marker.type = Marker.CUBE
				marker.action = Marker.ADD
				marker.scale = Vector3(0.2,0.2,0.2)
				marker.color.b = 0.5
				marker.color.a = 1.0
				marker.lifetime = lifetime
				marker_array.markers.append(marker)
			marker = markers[BOX]
			marker.pose = position_dict[BOX]
		for i, drone in enumerate(drones):
			if drone == BOX: continue
			if drone not in position_dict: continue
			if drone not in markers: 
				markers[drone] = Marker()
				marker = markers[drone]
				marker.header.frame_id = "world"
				marker.id = global_id
				global_id += 1
				marker.header.stamp = t
				marker.type = Marker.MESH_RESOURCE
				marker.mesh_resource = "package://drone_viz/src/drone.stl"
				marker.action = Marker.ADD
				marker.scale = Vector3(0.01,0.01,0.01)
				marker.color.g = 0.5
				marker.color.a = 1.0
				marker.lifetime = lifetime
				marker_array.markers.append(marker)
			marker = markers[drone]
			marker.pose = position_dict[drone]
			marker.pose = Pose()
			marker.pose.position.x = position_dict[drone].position.x
			marker.pose.position.y = position_dict[drone].position.y
			marker.pose.position.z = position_dict[drone].position.z
			pos_msg = position_dict[drone]
			euler = np.array(euler_from_quaternion([
						pos_msg.orientation.x,
						pos_msg.orientation.y,
						pos_msg.orientation.z, 
						pos_msg.orientation.w
					]))
			euler[0] += -np.pi/2
			quat = quaternion_from_euler(euler[0], euler[1], euler[2])
			marker.pose.orientation.x = quat[0]
			marker.pose.orientation.y = quat[1]
			marker.pose.orientation.z = quat[2]
			marker.pose.orientation.w = quat[3]

			#waypoint 
			if drone not in waypoint_dict: continue
			if drone not in waypoints:
				waypoints[drone] = Marker()
				marker = waypoints[drone]
				marker.header.frame_id = "world"
				marker.id = global_id
				global_id += 1
				marker.header.stamp = t
				marker.type = Marker.POINT
				marker.action = Marker.ADD
				marker.scale = Vector3(0.1,0.1,0.1)
				marker.color.r = 1
				marker.color.a = 1.0
				marker.lifetime = lifetime
				marker_array.markers.append(marker)
			marker = waypoints[drone]
			if marker.points[-1] != waypoints_dict[drone]:
				curr_waypoint = waypoints_dict[drone].data
				marker_point = Point()
				marker_point.x = curr_waypoint[0]
				marker_point.y = curr_waypoint[1]
				marker_point.z = curr_waypoint[2]
				marker.points.append(marker_point)

			#trajectories
			if drone not in trajectories:
				trajectories[drone] = Marker()
				marker = trajectories[drone]
				marker.header.frame_id = "world"
				marker.id = global_id
				global_id += 1
				marker.header.stamp = t
				marker.type = Marker.LINE_STRIP
				marker.action = Marker.ADD
				marker.scale = Vector3(0.1,0.1,0.1)
				marker.color.b = 1
				marker.color.a = 1.0
				marker.lifetime = lifetime
				marker_array.markers.append(marker)
			marker = trajectories[drone]
			if marker.points[-1] != waypoints_dict[drone]:
				curr_waypoint = waypoints_dict[drone].data
				marker_point = Point()
				marker_point.x = curr_waypoint[0]
				marker_point.y = curr_waypoint[1]
				marker_point.z = curr_waypoint[2]
				marker.points.append(marker_point)
			
			if BOX in drones and BOX in position_dict:
				# draw rope
				marker_name = "{}-{}".format(BOX, drone)
				if marker_name not in markers:
					markers[marker_name] = Marker()
					marker = markers[marker_name]
					marker.header.frame_id = "world"
					marker.id = global_id
					global_id += 1
					marker.header.stamp = t
					marker.type = Marker.LINE_STRIP
					marker.action = Marker.ADD
					marker.scale = Vector3(0.2,0.2,0.2)
					marker.color.r = 1
					marker.color.a = 1.0
					marker.lifetime = lifetime
					marker_array.markers.append(marker)
				marker = markers[marker_name]
				drone_pos = position_dict[drone]
				drone_point = Point(drone_pos.x, drone_pos.y, drone_pos.z)
				box_pos = position_dict[BOX]
				box_point = Point(box_pos.x, box_pos.y, box_pos.z)
				marker.points = [drone_point, box_point]
				

		pub.publish(marker_array)
		rate.sleep()

if __name__ == "__main__":
    try:
		drone_viz()
    except rospy.ROSInterruptException:
		pass
