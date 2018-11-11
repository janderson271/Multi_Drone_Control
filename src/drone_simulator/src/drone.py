#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Pose, Twist, Wrench
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float32MultiArray

def sim():
	rospy.init_node("drone")
	node_name = rospy.get_name()
	pos_pub = rospy.Publisher(node_name + "/position", Pose, queue_size=1)
	vel_pub = rospy.Publisher(node_name + "/velocity", Twist, queue_size=1)
	drone = Drone()
	control_sub = rospy.Subscriber(node_name + "/control", Float32MultiArray, drone.control_callback)
	control_sub = rospy.Subscriber(node_name + "/external_force", Wrench, drone.control_callback)
	rate = rospy.Rate(1/drone.dt)
	while not rospy.is_shutdown():
		#on fixed timestip: simulate the system and publish
		position, velocity = drone.sim_step()
		pos_pub.publish(position)
		vel_pub.publish(velocity)
		rate.sleep()


class Drone: 

	def __init__(self, length=0.5, width=0.5, mass=1, inertia_xx=.45, inertia_yy =0.45, inertia_zz=0.7, dt = 0.1, x0=np.zeros((12,1))):
		self.length = length
		self.width = width
		self.mass = mass
		self.inertia = np.diag([inertia_xx, inertia_yy, inertia_zz])
		self.dt = dt
		self.x = x0
		self.u = np.zeros((4,1))
		self.Fext = np.zeros((3,1))

	def sim_step(self):
		"""
		self.u = 4x1 np array [FL, FR, RL, RR]
		self.F_ext = 3x1 np array [x component, y component, z component]
		update self.x (12x1 state vector)
		use euler angles
		sets the internal state and returns pose and twist for the drone
		""" 
		#TODO: set self.x = f(x,u,F_ext)
		self.x[0] += 0.02
		self.x[1] += 0.02
		a = Pose()
		a.position.x = self.x[0]
		a.position.y = self.x[1]
		return a, Twist()

	def control_callback(self,control_msg):
		self.u = np.array(control_msg.data)

	def external_callback(self,external_msg):
		self.Fext = self.vectornp(external_msg.force)

        def vectornp(self, msg): return np.array([msg.x, msg.y, msg.z]) 

if __name__ == "__main__":
    try:
		sim()
    except rospy.ROSInterruptException:
		pass
