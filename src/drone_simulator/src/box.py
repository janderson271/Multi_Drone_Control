#!/usr/bin/env python
import numpy as np
import rospy
import rosnode
import ipdb
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose, Wrench, Vector3

def skew(x):
	return np.array([[0, -x[2], x[1]], [x[2], 0, x[0]], [x[1], x[0], 0]])

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

	params_dict = dict(rope_length=1, mass=0.03, dt=0.1, x0=[0.]*3, v0=[0.]*3)
	params = get_and_set_params(node_name, params_dict)
	params['x0'] = np.array(params['x0'], dtype=np.float64).reshape((1,3))
	params['v0'] = np.array(params['v0'], dtype=np.float64).reshape((1,3))
	
	# initialize box
	box = Box(**params)
	# box = Box()
	node_names = rosnode.get_node_names()

	box.num_drones = sum(1 for i in node_names if i.startswith('/drone'))

	# get drone positions
	for drone_node in node_names:
		if drone_node.startswith('/drone'):
			rospy.Subscriber(drone_node + '/position', Pose, box.drone_callback)

	# get time
	time_sub = rospy.Subscriber('god/time', Int32, box.timer_callback)


	rate = rospy.Rate(1/box.dt)
	while not rospy.is_shutdown():
		if box.global_time > box.time:
			forces, position = box.sim_step()
			# publish Fext
			for i in range(0, len(forces)):
				# Fext = box.toDrone(forces[i], box.drone_orientations[i])
				Fext = forces[i]
				force_pub = rospy.Publisher(node_names[i] + '/external_force', Wrench, queue_size=1)
				force_pub.publish(Fext)
			# publish box position
			box_pos_pub = rospy.Publisher('box/position', Pose, queue_size=1)
			box_pos_pub.publish(position)

		rate.sleep()
		
	clear_params(node_name, params_dict)

class Box:  

	def __init__(self, rope_length=1, mass=0.03, dt = 0.1, x0=np.zeros((1,3)), v0=np.ones((1,3))):
		self.rope_length = rope_length
		self.mass = mass
		self.dt = dt
		self.pos = x0.T
		self.vel = v0.T
		self.drone_positions = []
		self.drone_orientations = []
		self.time = 0
		self.global_time = 0
		self.num_drones = 0
		self.k = 0.1
		self.x_last = 0

	def reset(self):
		self.drone_positions = []

	def sim_step(self):
		'''
		self.drone_positions = [[drone_1], [drone_2], ... [drone_n]]
			where drone_i = [i.x, i.y, i.z]
		return [[wrench_1], [wrench_2], ... [wrench_n]]
			where wrench_i.force.x = fext_x on drone_i, in drone_i frame
		'''

		if self.drone_positions == []:
			self.time += 1
			return [Wrench()]*self.num_drones, self.get_pose()

		# forces on box
		Fg = np.array([[0, 0, -self.mass*9.81]]).T
		# ipdb.set_trace()
		sum_z = sum((row[2] - self.pos[2]) for row in self.drone_positions if np.linalg.norm(self.pos - row)>=self.rope_length)
		if sum_z == 0:
			self.time += 1
			return [Wrench()]*self.num_drones, self.get_pose()

		T = -Fg[2]/sum_z
		Fd = np.zeros((1,3)).T
		forces = []

		for drone_pos in self.drone_positions:
			F = Wrench()
			# drone not supporting box
			if np.linalg.norm(self.pos - drone_pos) < self.rope_length:
				F.force.x = 0
				F.force.y = 0
				F.force.z = 0
			# drone supporting box
			else:
				# f = T*(self.pos - drone_pos) #/np.linalg.norm(self.pos - drone_pos)
				f = -self.k * (np.linalg.norm(self.pos - drone_pos) - self.rope_length) * (self.pos - drone_pos) / np.linalg.norm(self.pos - drone_pos)
				damping = (np.linalg.norm(self.pos - drone_pos) - self.x_last)/self.dt
				self.x_last = np.linalg.norm(self.pos - drone_pos)
				F.force.x = -f[0]
				F.force.y = -f[1]
				F.force.z = -f[2]
				Fd += f
			forces += [F]	

		# all forces
		c = 0
		Fs = Fg + Fd - c*damping

		# acceleration
		accel = Fs / self.mass	

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
		return np.array([[msg.x, msg.y, msg.z]]).T

	def npvector(self, msg):
		return Vector3(msg[0], msg[1], msg[2])
   
	def drone_callback(self, position_msg):
		if len(self.drone_positions) == self.num_drones:
			self.drone_positions = []
			self.drone_orientations = []

		self.drone_positions += [self.vectornp(position_msg.position)]
		self.drone_orientations += [position_msg.orientation]

	def timer_callback(self, time_msg):
		self.global_time = time_msg.data

	def toDrone(self, v, p):
		# convert vector v to coordinates of drone with quaternion p
		q0 = p.w;
		qv = self.vectornp(p)
		f = self.vectornp(v.force)
		T = q0*q0*np.eye(3) - (qv.T*qv).dot(np.eye(3)) + 2*(qv*qv.T) - 2*q0*skew(qv)
		w = Wrench()
		w.force = self.npvector(T.dot(f))
		return w


if __name__ == '__main__':
    try:
		sim()
    except rospy.ROSInterruptException:
		pass
