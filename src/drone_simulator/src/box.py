#!/usr/bin/env python
import numpy as np
import rospy
import rosnode
import ipdb
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose, Wrench, Vector3

def get_and_set_params(node_name, params):
	# params is a dict from param_name -> default_value, None if no default value
	vals = {}
	for param, default in params.items():
		if default is None:
			vals[param] = rospy.get_param('{}/{}'.format(node_name, param))
		else:
			if not rospy.has_param('{}/{}'.format(node_name, param)):
				rospy.set_param('{}/{}'.format(node_name, param), default)
			vals[param] = rospy.get_param('{}/{}'.format(node_name, param))
	return vals

def clear_params(node_name, params):
	for k in params:
		try:
			rospy.delete_param('{}/{}'.format(node_name, k))
		except KeyError:
			print('value {} not set'.format(k))
	return True

def sim():
	node_name = 'box'
	rospy.init_node(node_name)

	params_dict = dict(rope_length=1, mass=0.01, dt=0.1, x0=[0.]*3, v0=[0.]*3)
	params = get_and_set_params(node_name, params_dict)
	params['x0'] = np.array(params['x0'], dtype=np.float64).flatten()
	params['v0'] = np.array(params['v0'], dtype=np.float64).flatten()
	
	box = Box(**params)
	print(np.array(params['x0'], dtype=np.float64).flatten())
	node_names = rosnode.get_node_names()
	drone_node_names = []

	node_name = rospy.get_name()
	num_drones = rospy.get_param(node_name + "/num_drones")

	box.num_drones = num_drones #sum(1 for i in node_names if i.startswith('/drone'))

	# get drone positions
	node_names = ["/drone_{}".format(i+1) for i in range(num_drones)]
	for drone_node in node_names:
		if drone_node.startswith('/drone'):
			rospy.Subscriber(drone_node + '/position', Pose, box.drone_callback)
			drone_node_names += [drone_node]

	# get time
	time_sub = rospy.Subscriber('god/time', Int32, box.timer_callback)

	# sim
	rate = rospy.Rate(1/box.dt)
	while not rospy.is_shutdown():
		#import ipdb; ipdb.set_trace()
		if box.global_time > box.time:
			forces, position = box.sim_step()
			for i in range(0, len(forces)):
				Fext = forces[i]
				force_pub = rospy.Publisher(drone_node_names[i] + '/external_force', Wrench, queue_size=1)
				force_pub.publish(Fext)
			box_pos_pub = rospy.Publisher('box/position', Pose, queue_size=1)
			box_pos_pub.publish(position)

		rate.sleep()
		
	clear_params(node_name, params_dict)

class Box:  

	def __init__(self, rope_length=1, mass=0.01, dt = 0.1, x0=np.array([0, 0.5, 0]), v0=np.ones(3)):
		self.rope_length = rope_length				# length of "ropes" [m]
		self.mass = mass							# mass of box [kg]
		self.dt = dt 								# time step [s]
		self.pos = x0					 			# starting position (x,y,z) [m]
		self.vel = v0 								# current velocity (vx,vy,vz) [m/s]
		self.drone_positions = [] 					# list of drone positions 
		self.time = 0 								# box time 
		self.global_time = 0 					    # global time
		self.num_drones = 0 						# number of drones
		self.k = 0.3 								# spring constant
		self.c = 0.05 								# damping constant

	def reset(self):
		self.drone_positions = []

	def sim_step(self):
		'''
		self.drone_positions = [[drone_1], [drone_2], ... [drone_n]]
			where drone_i = [i.x, i.y, i.z]
		return [[wrench_1], [wrench_2], ... [wrench_n]]
			where wrench_i.force.x = fext_x on drone_i, in drone_i frame
		'''

		# no drone positions
		if self.drone_positions == []:
			self.time += 1
			return [Wrench()]*self.num_drones, self.get_pose()

		# forces on box
		Fg = np.array([0, 0, -self.mass*9.81]) 	
		Fs = np.zeros(3)
		Fd = self.c * self.vel
		forces = []

		# "spring" forces
		for drone_pos in self.drone_positions:
			W = Wrench()
			# drone not supporting box
			if np.linalg.norm(self.pos - drone_pos) < self.rope_length:
				W.force.x = 0
				W.force.y = 0
				W.force.z = 0
			# drone supporting box
			else:
				f = -self.k * (np.linalg.norm(self.pos - drone_pos) - self.rope_length) * (self.pos - drone_pos) / np.linalg.norm(self.pos - drone_pos)
				W.force.x = -f[0]
				W.force.y = -f[1]
				W.force.z = -f[2]
				Fs += f
			forces += [W]	

		# total force
		F = Fg + Fs - Fd

		# acceleration
		accel = F / self.mass	

		# step forward position, velocity
		self.pos = self.pos + self.vel * self.dt
		self.vel = self.vel + accel * self.dt

		# enforce ground
		self.pos[2] = np.max([self.pos[2], 0])

		# return forces, new box position
		self.time += 1
		return forces, self.get_pose()

	def get_pose(self):
		pose = Pose()
		pose.position.x = self.pos[0]
		pose.position.y = self.pos[1]
		pose.position.z = self.pos[2]
		pose.orientation.x = 0
		pose.orientation.y = 0
		pose.orientation.z = 0
		pose.orientation.w = 1
		return pose

	def vectornp(self, msg): 
		return np.array([msg.x, msg.y, msg.z])

	def npvector(self, msg):
		return Vector3(msg[0], msg[1], msg[2])
   
	def drone_callback(self, position_msg):
		if len(self.drone_positions) == self.num_drones:
			self.drone_positions = []
		self.drone_positions += [self.vectornp(position_msg.position)]

	def timer_callback(self, time_msg):
		self.global_time = time_msg.data

if __name__ == '__main__':
    try:
		sim()
    except rospy.ROSInterruptException:
		pass
