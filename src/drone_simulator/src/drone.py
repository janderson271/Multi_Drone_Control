#!/usr/bin/env python
import numpy as np
import rospy
import ipdb
from geometry_msgs.msg import Pose, Twist, Wrench
from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix, inverse_matrix
from std_msgs.msg import Float32MultiArray, Int32

def get_and_set_params(node_name, params):
	# params is a dict from param_name -> default_value, None if no default value
	vals = {}
	for param, default in params.items():
		if default is None:
			vals[param] = rospy.get_param("{}/{}".format(node_name, param))
		else:
			if not rospy.has_param("{}/{}".format(node_name, param)):
				rospy.set_param("{}/{}".format(node_name, param), default)
			vals[param] = rospy.get_param("{}/{}".format(node_name, param))
	return vals

def clear_params(node_name, params):
	for k in params:
		try:
			rospy.delete_param("{}/{}".format(node_name, k))
		except KeyError:
			print("value {} not set".format(k))
	return True

def sim():
	rospy.init_node("drone")
	node_name = rospy.get_name()
	print(node_name)
	pos_pub = rospy.Publisher(node_name + "/position", Pose, queue_size=1)
	vel_pub = rospy.Publisher(node_name + "/velocity", Twist, queue_size=1)
	
	params_dict = dict(length=0.5, width=0.5, mass=1, inertia_xx=.45, inertia_yy =0.45, inertia_zz=0.7, k=0.3, dt = 0.1, x0=[0.] * 12)
	params = get_and_set_params(node_name, params_dict)
	print(len(params["x0"]))
	params["x0"] = np.array(params["x0"], dtype=np.float64).reshape((12, 1))
	drone = Drone(**params)

	if rospy.has_param("/{}/{}".format(node_name, "u_init")):
		param = rospy.get_param("/{}/{}".format(node_name, "u_init"))
		param = np.array(param).reshape((4, 1))
		drone.u = param
		print("setting initial u to ", drone.u)
	
	control_sub = rospy.Subscriber(node_name + "/control", Float32MultiArray, drone.control_callback)
	control_sub = rospy.Subscriber(node_name + "/external_force", Wrench, drone.control_callback)

	time_sub = rospy.Subscriber("god/time", Int32, drone.timer_callback)
	
	rate = rospy.Rate(1/drone.dt)
	position = drone.get_pose()
	velocity = drone.get_twist()

	while not rospy.is_shutdown():
		#on fixed timestip: simulate the system and publish
		if drone.global_time > drone.time:
			position, velocity = drone.sim_step()
			pos_pub.publish(position)
			vel_pub.publish(velocity)
		rate.sleep()
	clear_params(node_name, params_dict)


class Drone: 

	def __init__(self, length=0.5, width=0.5, mass=1, inertia_xx=.45, inertia_yy =0.45, inertia_zz=0.7, k=0.3, dt = 0.1, x0=np.zeros((12,1))):
		self.length = length
		self.width = width
		self.mass = mass
		self.inertia = np.diag([inertia_xx, inertia_yy, inertia_zz])
		self.k = k
		self.dt = dt
		self.x = x0
		self.u = np.zeros((4,1))
		self.Fext = np.zeros((3,1))
		self.time = 0
		self.global_time = 0


	def sim_step(self):
		"""
		self.u = 4x1 np array [FL, FR, RL, RR]
		self.F_ext = 3x1 np array [x component, y component, z component]
		update self.x (12x1 state vector)
		use euler angles
		sets the internal state and returns pose and twist for the drone
		x_subset = x, y, z, roll, pitch, yaw
		x = x_subset, d(x_subset)/dt 
		""" 
		
		# Calculate Forces and moments
		g = 9.81
		Fg = np.array([[0], [0], [-self.mass * g]])
		Ft = np.array([[0], [0], [np.sum(self.u)]])
		
		Fs = Fg + self.Fext + self.toGlobal(Ft).reshape(3,1) 
		

		Mz = np.array([0,0, self.k * (self.u[0] - self.u[1] + self.u[2] - self.u[3])]) 
		Sfl = self.width * np.array([1, 1 ,0])
		Sfr = self.width * np.array([1, -1 ,0])
		Srl = self.width * np.array([-1, -1 ,0])
		Srr = self.width * np.array([-1, 1 ,0])

		M = Mz + (np.cross(Sfl, np.array([0, 0, self.u[0]])) 
			+ np.cross(Sfr, np.array([0, 0, self.u[1]])) 
			+ np.cross(Srl, np.array([0, 0, self.u[2]]))
			+ np.cross(Srr, np.array([0, 0, self.u[3]])))
		
		# Calculate accelerations
		a_lin = (1 / self.mass) * Fs
		a_ang = np.dot(np.linalg.inv(self.inertia), (M - np.cross(self.x[9:].T, np.dot(self.inertia, self.x[9:]).T)).T)

		# advance dynamics
		self.x[0:6] = self.x[0:6] + self.x[6:] * self.dt
		self.x[6:9] = self.x[6:9] + a_lin * self.dt
		self.x[9:] = self.x[9:] + a_ang * self.dt

		# Enforce that the ground exists
		self.x[2] = np.max(self.x[2], 0)
		if self.x[2] == 0:
			self.x[8] = np.max(self.x[8], 0)

		# return messages
		position = self.get_pose()
		velocity = self.get_twist()

		self.time += 1

		return position, velocity

	def get_pose(self):
		position = self.x[0:3]
		orientation = quaternion_from_euler(self.x[3], self.x[5], self.x[6])
		pose = Pose()
		pose.position.x = position[0]
		pose.position.y = position[1]
		pose.position.z = position[2]
		pose.orientation.x = orientation[0]
		pose.orientation.y = orientation[1]
		pose.orientation.z = orientation[2]
		pose.orientation.w = orientation[3]
		return pose

	def get_twist(self):
		twist = Twist()
		linear = self.x[6:9]
		angular = self.x[9:]
		twist.linear.x = linear[0]
		twist.linear.y = linear[1]
		twist.linear.z = linear[2]
		twist.angular.x = angular[0]
		twist.angular.y = angular[1]
		twist.angular.z = angular[2]
		return twist
		
	def control_callback(self,control_msg):
		self.u = np.array(control_msg.data)

	def external_callback(self,external_msg):
		self.Fext = self.vectornp(external_msg.force)

	def timer_callback(self, time_msg):
		self.global_time = time_msg.data

	def vectornp(self, msg): return np.array([msg.x, msg.y, msg.z]) 

	def toLocal(self, vector):
		wx = self.x[3] # roll
		wy = self.x[4] # pitch
		wz = self.x[5] # yaw
		Rx = np.array([[1, 0, 0],[0, np.cos(wx), -np.sin(wx)],[0, np.sin(wx), np.cos(wx)]])
		Ry = np.array([[np.cos(wy), 0, np.sin(wy)],[0, 1, 0],[-np.sin(wy), 0, np.cos(wy)]])
		Rz = np.array([[np.cos(wz), -np.sin(wz), 0],[np.sin(wz),np.cos(wz), 0],[0,0,1]])
		R = np.linalg.inv(np.dot(Rx, np.dot(Ry,Rz)))
		return np.dot(R, vector)


	def toGlobal(self, vector):
		wx = self.x[3] # roll
		wy = self.x[4] # pitch
		wz = self.x[5] # yaw
		Rx = np.array([[1, 0, 0],[0, np.cos(wx), -np.sin(wx)],[0, np.sin(wx), np.cos(wx)]])
		Ry = np.array([[np.cos(wy), 0, np.sin(wy)],[0, 1, 0],[-np.sin(wy), 0, np.cos(wy)]])
		Rz = np.array([[np.cos(wz), -np.sin(wz), 0],[np.sin(wz),np.cos(wz), 0],[0,0,1]])
		R = np.dot(Rx, np.dot(Ry,Rz))
		return np.dot(R, vector)





if __name__ == "__main__":
    try:
		sim()
    except rospy.ROSInterruptException:
		pass
