#!/usr/bin/env python
import numpy as np
import rospy
import rosnode
from geometry_msgs.msg import Pose, Wrench
from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix, inverse_matrix

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
			Fext = box.toDrone(wrenches[i], drone_positions[i])
			rospy.Publisher(drone_node_names[i] + '/external_force', Wrench, queue_size=1)
			force_pub.publish(Fext)

		rate.sleep()

class Box: 

	def __init__(self, rope_length=1, mass=10, dt = 0.1, x0=np.zeros((3))):
		self.rope_length = rope_length
		self.mass = mass
		self.dt = dt
		self.x = x0
		self.drone_positions = []

	def sim_step(self):
		"""
		self.drone_positions = [[drone_1], [drone_2], ... [drone_n]]
			where drone_i = [i.x, i.y, i.z]
		return [[wrench_1], [wrench_2], ... [wrench_n]]
		""" 

		Fg = np.array([0, 0, -self.mass*9.81])
		v = 0
		polygon = []

		# drones that are supporting the box
		for p in self.drone_positions:
			vect = self.x - np.array(p)
			dist = np.norm(vect)
			if dist >= self.rope_length:
				v += [vect]
				polygon += [p]
		v0 = self.mass*9.8/v
		wrenches = []

		for p in self.drone_positions:
			w = Wrench()
			if np.norm(self.x - np.array(p)) < 1:
				w.force.x = 0
				w.force.y = 0
				w.force.z = 0
			else:
				f = v0*(self.x - np.array(p))/np.norm(self.x)
				w.force.x = f[0]
				w.force.y = f[1]
				w.force.z = f[2]
			wrenches += [w]		

		# current position of box
		box_x, box_y, _ = np.mean(polygon, axis=0) # center of polygon of drones
		r = np.linalg.norm(np.array(polygon[0])-np.array([box_x, box_y])) # xy distance from box to drone
		box_z = np.sqrt(pow(r,2) + pow(self.rope_length, 2)) 
		self.x = np.array([box_x, box_y, box.z])

		# force_mag = ((self.mass * 9.8)*self.rope_length/box_z)/self.num_drones

		
		# for p in drone_positions:
		# 	w = Wrench()
		# 	d = np.array([box_x, box_y, box_z])-np.array(p)
		# 	w.force.x, w.force.y, w.force.z = force_mag*d/np.linalg.norm(d)
			
		return wrenches
	def vectornp(self, msg): 
		return np.array([[msg.x, msg.y, msg.z]])

	def skew(x):
    	return np.array([[0, -x[2], x[1]], [x[2], 0, x[0]], [x[1], x[0], 0]])
   
	def drone_callback(self, position_msg):
		self.drone_positions += [self.vectornp(position_msg.position)]
		self.num_drones = len(self.drone_positions)

	def toDrone(self, v, p):
		"""
		convert vector v to coordinates of drone with pose p
		"""
		q0 = p.quaternion.w;
		qv = self.vectornp(p.quaternion)
		T = q0*q0*np.eye(3) - (qv.T*qv).dot(np.eye(3)) + 2*(qv*qv.T) - 2*q0*skew(qv)
		return T.dot(v)


if __name__ == "__main__":
    try:
		sim()
    except rospy.ROSInterruptException:
		pass
