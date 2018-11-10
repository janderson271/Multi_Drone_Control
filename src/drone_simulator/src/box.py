#!/usr/bin/env python
import numpy as np
import rospy
import rosnode
from geometry_msgs.msg import Pose, Twist, Wrench
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float32MultiArray

def sim():
	rospy.init_node("box")
	box = Box()
	drone_node_names = rosnode.get_node_names()
	for node_name in drone_node_names:
		if node_name.startswith('/drone'):
			rospy.Subscriber(node_name + '/position', Pose, box.drone_callback)
	rate = rospy.Rate(1/box.dt)
	while not rospy.is_shutdown():
		#on fixed timestep: simulate the system and publish
		wrenches = drone.sim_step()
		for i in range(0, len(wrenches)):
			w = wrenches[i]
			node_name = drone_node_names[i]
			rospy.Publisher(node_name + '/external_force', Wrench, queue_size=1)
			force_pub.publish(external_force)
		rate.sleep()


class Box: 

	def __init__(self, rope_length=1, mass=10, dt = 0.1):
		self.rope_length = rope_length
		self.mass = mass
		self.dt = dt

	def sim_step(self):
		"""
		self.drone_positions = [[drone_1], [drone_2], ... [drone_n]]
			where drone_i = [i.x, i.y, i.z]
		return [[wrench_1], [wrench_2], ... [wrench_n]]
		assume drones in valid position (xy polygon with num_drone vertices), all ropes in tension of length rope_length
		""" 
		box_x, box_y, _ = np.mean(self.drone_positions, axis=0) # center of polygon of drones
		r = np.linalg.norm(np.array(self.drone_positions[0])-np.array([box_x, box_y])) # xy distance from box to drone
		box_z = np.sqrt(pow(r,2) + pow(self.rope_length, 2)) 
		force_mag = ((self.mass * 9.8)*self.rope_length/box_z)/self.num_drones

		wrenches = []
		for p in drone_positions:
			w = Wrench()
			d = np.array([box_x, box_y, box_z])-np.array(p)
			w.force.x, w.force.y, w.force.z = force_mag*d/np.linalg.norm(d)
			wrenches += [w]
		return wrenches

	def drone_callback(self, position_msg):
		self.drone_positions += [[position_msg.position.x, position_msg.position.y, position_msg.position.z]]
		self.num_drones = len(self.drone_positions)

if __name__ == "__main__":
    try:
		sim()
    except rospy.ROSInterruptException:
		pass
