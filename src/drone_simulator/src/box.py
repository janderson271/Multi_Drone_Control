#!/usr/bin/env python
import numpy as np
import rospy
import rosnode
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose, Wrench, Vector3, Twist

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

	params_dict = dict(rope_length=1, mass=0.01, dt=0.1, x0=[0.]*3, v0=[0.]*3, num_drones=0)
	params = get_and_set_params(node_name, params_dict)
	params['x0'] = np.array(params['x0'], dtype=np.float64).flatten()
	params['v0'] = np.array(params['v0'], dtype=np.float64).flatten()

	node_name = rospy.get_name()
	num_drones = rospy.get_param(node_name + "/num_drones")

	drones = []
	for i in range(1, num_drones + 1):
		drone_node = "drone_" + str(i)
		drones.append(PubObj(drone_node))

	box = Box(**params)
	box.drones = drones

	time_sub = rospy.Subscriber('god/time', Int32, box.timer_callback)
	box_pos_pub = rospy.Publisher('box/position', Pose, queue_size=1)
	rate = rospy.Rate(1/box.dt)

	while not rospy.is_shutdown():
		if box.global_time > box.time:
			position = box.sim_step()
			box_pos_pub.publish(position)

		rate.sleep()
		
	clear_params(node_name, params_dict)

class PubObj:

	def __init__(self, drone_node): 
		self.pos_sub = rospy.Subscriber(drone_node + '/position', Pose, self.drone_callback)
		self.vel_sub = rospy.Subscriber(drone_node + '/velocity', Twist, self.drone_vel_callback)
		self.pub = rospy.Publisher(drone_node + '/external_force', Wrench, queue_size=1)
		self.pos = np.zeros(3)
		self.vel = np.zeros(3)

	def drone_callback(self, position_msg):
		self.pos = self.vectornp(position_msg.position)

	def drone_vel_callback(self, position_msg):
		self.vel = self.vectornp(position_msg.linear)

	def vectornp(self, msg): 
		return np.array([msg.x, msg.y, msg.z])

	def pub_force(self, W): 
		self.pub.publish(W)

class Box:  

	def __init__(self, rope_length=1.5, mass=0.01, dt = 0.1, x0=np.array([0, 0.5, 0]), v0=np.ones(3), num_drones=0):
		self.rope_length = rope_length				# length of "ropes" [m]
		self.mass = mass							# mass of box [kg]
		self.dt = dt 								# time step [s]
		self.pos = x0					 			# starting position (x,y,z) [m]
		self.vel = v0 								# current velocity (vx,vy,vz) [m/s]
		self.time = 0 								# box time 
		self.global_time = 0 					    # global time
		self.num_drones = 0 						# number of drones
		self.k = 0.11								# spring constant
		self.c = 0.0222 							# damping constant
		self.drones = []

	def sim_step(self):
		'''
		publishes external force acting on drone in world frame
		returns box position in world frame
		'''
				
		Fg = np.array([0, 0, -self.mass*9.81])
		Fs = np.zeros(3)							

		for drone in self.drones:
			drone_pos = drone.pos
			drone_vel = drone.vel
			W = Wrench()
			# drone not supporting box
			if np.linalg.norm(self.pos - drone_pos) < self.rope_length:
				W.force.x = 0
				W.force.y = 0
				W.force.z = 0
			# drone supporting box
			else:
				d = -self.c * (np.linalg.norm(self.vel - drone_vel)) * (self.vel - drone_vel) / np.linalg.norm(self.vel - drone_vel)
				f = -self.k * (np.linalg.norm(self.pos - drone_pos) - self.rope_length) * (self.pos - drone_pos) / np.linalg.norm(self.pos - drone_pos)
				W.force.x = -f[0] - d[0]
				W.force.y = -f[1] - d[1]
				W.force.z = -f[2] - d[2]
				Fs += f + d
			drone.pub_force(W)	

		# total force
		F = Fg + Fs 

		# acceleration
		accel = F / self.mass	

		# step forward position, velocity
		self.pos = self.pos + self.vel * self.dt
		self.vel = self.vel + accel * self.dt

		# enforce ground
		self.pos[2] = np.max([self.pos[2], 0])

		# return forces, new box position
		self.time += 1
		return self.get_pose()

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

	def timer_callback(self, time_msg):
		self.global_time = time_msg.data

if __name__ == '__main__':
    try:
		sim()
    except rospy.ROSInterruptException:
		pass
