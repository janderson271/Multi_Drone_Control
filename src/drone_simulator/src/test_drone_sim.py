import numpy as np
import rospy
import ipdb
from geometry_msgs.msg import Pose, Twist, Wrench
from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix, inverse_matrix
from std_msgs.msg import Float32MultiArray, Int32

from drone import Drone 

drone = Drone()

# test 1
drone.x = np.zeros((12,1))
drone.u = np.array([0.1,0.1,0,0])
drone.sim_step()
print((drone.x[0:10].all() == 0 and drone.x[11] == 0 and drone.x[10] < 0))

# test 2
drone2 = Drone()
drone2.x = np.zeros((12,1))
drone2.u = np.array([0,1,1,0])
drone2.sim_step()
print(drone2.x[0:9].all() == 0 and drone2.x[10:].all() == 0 and drone2.x[9] < 0)

# test 3
drone3 = Drone()
drone3.x = np.zeros((12,1))
drone3.u = np.array([0.1,0,0.1,0])
print(drone3.x)
drone3.sim_step()
print(drone3.x[0:11].all() == 0 and drone3.x[11] > 0)
ipdb.set_trace()
print(drone3.x)
drone3.sim_step()
print(drone3.x)



