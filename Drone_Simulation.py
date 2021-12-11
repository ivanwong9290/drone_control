import numpy as np
import matplotlib.pyplot as plt
# import scipy.io as sio



# gain_matrix_data = sio.loadmat('../Gain Matrix.mat')
# K = gain_matrix_data['K']

flight_data = np.load('stateData.npy', allow_pickle = True)
create_data_array(flight_data)
drone_states = np.load('Drone_Flight_Data_Thrust.npy')

drone_states[:, 0:3] = drone_states[:, 0:3] - drone_states[0, 0:3]
drone_states[:, 3:6] = drone_states[:, 3:6] * np.pi / 180
drone_states[:, 9:12] = drone_states[:, 9:12] * np.pi / 180

plt.plot(drone_states[:, 0:3])
plt.ylabel('Position (m)')
plt.title('Position vs Time')
plt.legend(['X', 'Y', 'Z'])
plt.grid()
plt.show()
# plt.plot(drone_states[:, 3:6])
# plt.ylabel('Orientation (rad)')
# plt.title('Orientation vs Time')
# plt.legend(['Phi', 'Theta', 'Psi'])
# plt.grid()
# plt.show()
plt.plot(drone_states[:, 6:9])
plt.ylabel('Linear Velocity (m/s)')
plt.title('Linear Velocity vs Time')
plt.legend(['X_dot', 'Y_dot', 'Z_dot'])
plt.grid()
plt.show()
# plt.plot(drone_states[:, 9:12])
# plt.ylabel('Angular Velocity (rad/s)')
# plt.title('Angular Velocity vs Time')
# plt.legend(['Phi_dot', 'Theta_dot', 'Psi_dot'])
# plt.grid()
# plt.show()

# plt.plot(drone_states[:, 0])
# plt.ylabel('Thrust')
# plt.title('Thrust vs Time')
# plt.legend(['Thrust'])
# plt.grid()
# plt.show()

# plt.plot(drone_states[:, 1:4])
# plt.ylabel('Moment')
# plt.title('Moment vs Time')
# plt.legend(['Mx', 'My', 'Mz'])
# plt.grid()
# plt.show()

