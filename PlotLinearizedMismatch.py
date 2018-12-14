import numpy as np 
import matplotlib.pyplot as plt 
import sys
sys.path.insert(0, './src/drone_simulator/src')
from drone import Drone 
sys.path.insert(0, './src/drone_controller/src')
from MPCcontroller import MPCcontroller

Q = np.array([1.,1,1,1,1,1,1,1,1,1,1,1])
P = np.array([1.,1,1,1,1,1,1,1,1,1,1,1])
R = np.array([0.1,0.1,0.1,0.1])
u_init = np.array([0.00, 0.000, 0.000, 0.000])
ts = 0.1
xbar = np.array([1.,1.,1,0,0,0,0,0,0.1,0,0,0])
ubar = np.array([0.,0,0,0])
x0 = np.array([0.,0,0,0,0,0,0,0,0,0,0,0])

controller = MPCcontroller(ts, P, Q, R, xbar, ubar, x0)
drone = Drone(length=0.033, width=0.033, mass=0.032, inertia_xx=16e-6, inertia_yy =16e-6, inertia_zz=29e-6, k=0.01, dt = 0.1, x0=np.array([0.] * 12).reshape(12,1))
controller.actuate(x0)

Uvals = np.array(controller.U.value)
print(Uvals.shape)
lin_states = np.array(controller.X.value)
non_lin_states = np.zeros(lin_states.shape)
non_lin_states[:,0] = np.copy(drone.x).flatten()

for i in range(0,Uvals.shape[1]):
	drone.u = Uvals[:,i]
	drone.sim_step()
	non_lin_states[:,i+1] = np.copy(drone.x).flatten()

tvec = np.linspace(0, 3, 31)

plt.figure()
def plot_state(index1, index2, name, subplot):
	plt.subplot(2, 2, subplot)
	plt.plot(tvec, lin_states[index1,:], "k", label="linearized", linewidth=2)
	plt.plot(tvec, non_lin_states[index2,:], "r--", label="non-linear", linewidth=2)
	plt.xlabel("time in seconds")
	plt.ylabel("X in meters")
	plt.title("{} vs. t".format(name))
	plt.legend(loc=0)
	plt.grid()
plot_state(0, 0, "X", 1)
plot_state(1, 1, "Y", 2)
plot_state(2, 2, "Z", 3)
plot_state(8, 5, "Yaw", 4)
plt.tight_layout()

# plt.figure()
# plt.plot(tvec, lin_states[1,:], "k", label="linear")
# plt.plot(tvec, non_lin_states[1,:], "r--", label="non-linear")
# plt.xlabel("time")
# plt.ylabel("Y")
# plt.title("Y vs. t")
# plt.legend(loc=0)

# plt.figure()
# plt.plot(tvec, lin_states[2,:], "k", label="linear")
# plt.plot(tvec, non_lin_states[2,:], "r--", label="non-linear")
# plt.xlabel("time")
# plt.ylabel("Z")
# plt.title("Z vs. t")
# plt.legend(loc=0)

# plt.figure()
# plt.plot(tvec, lin_states[8,:], "k", label="linear")
# plt.plot(tvec, non_lin_states[5,:], "r--", label="non-linear")
# plt.xlabel("time")
# plt.ylabel("Yaw")
# plt.title("Yaw vs. t")
# plt.legend(loc=0)
# plt.grid


plt.show()

