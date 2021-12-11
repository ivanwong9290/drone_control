import logging
import time
# import keyboard
import numpy as np
from icecream import ic
import matplotlib.pyplot as plt
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.commander import Commander

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E7E7'


# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def create_data_array(test_data):
    states = []
    for i in range(test_data.shape[0]):
        state = []
        for key in test_data[i].keys():
            state.append(test_data[i][key])
        states.append(state)

    states = np.asarray(states)
    np.save('Drone_Flight_Data_Thrust.npy', states)

def plotGraphs(drone_states):
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

    return

def simple_log(scf, logconf):

    with SyncLogger(scf, lg_stab) as logger:
        flightTime = 30
        startTime = time.time() 
        endTime = startTime + flightTime
        cf = scf.cf
        stateData = []
        x_ref, y_ref, z_ref = 0, 0, 0.3

        for log_entry in logger:
            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]
            stateData.append(data)

            # print('[%d]: %s' % (timestamp, data))
            x_curr, y_curr, z_curr = data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z']
            x_error, y_error, z_error = x_ref - x_curr, y_ref - y_curr, z_ref - z_curr

            # ic(x_error, y_error, z_error)

            # Hover
            # if(time.time() < endTime - 1): cf.commander.send_hover_setpoint(0, 0, 0, z_ref)
            # else: cf.commander.send_hover_setpoint(0, 0, 0, 0.1)
            
            # Control Position
            if(time.time() <= startTime + 2.5): cf.commander.send_position_setpoint(x_ref, y_ref, z_ref, 0)
            elif(time.time() < endTime - 2.5): cf.commander.send_position_setpoint(x_ref + x_error, y_ref + y_error, z_ref + z_error, 0)
            elif(time.time() < endTime - 2): cf.commander.send_position_setpoint(x_ref + x_error, y_ref + y_error, z_ref - 0.5, 0)
            elif(time.time() < endTime - 1): cf.commander.send_position_setpoint(x_ref + x_error, y_ref + y_error, z_ref - 1.0, 0)
            # else: cf.commander.send_position_setpoint(x_ref + x_error, y_ref + y_error, z_ref, 0)
            else: cf.commander.send_position_setpoint(x_ref, y_ref, z_ref -1.25, 0)

            if time.time() > endTime:
                print("Exiting Logging after ", flightTime, " s")
                break

        stateData = np.asarray(stateData)
        np.save("stateData.npy",stateData)

        print("Saved Data")

        flight_data = np.load('stateData.npy', allow_pickle = True)
        create_data_array(flight_data)
        drone_states = np.load('Drone_Flight_Data_Thrust.npy')

        plotGraphs(drone_states)

        return


if __name__ == '__main__':

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)


    # Defining logging configs
    lg_stab = LogConfig(name='state', period_in_ms=100)
    lg_stab.add_variable('stateEstimate.x', 'FP16')
    lg_stab.add_variable('stateEstimate.y', 'FP16')
    lg_stab.add_variable('stateEstimate.z', 'FP16')
    # lg_stab.add_variable('stateEstimate.roll', 'FP16')
    # lg_stab.add_variable('stateEstimate.pitch', 'FP16')
    # lg_stab.add_variable('stateEstimate.yaw', 'FP16')
    # lg_stab.add_variable('stateEstimate.vx', 'FP16')
    # lg_stab.add_variable('stateEstimate.vy', 'FP16')
    # lg_stab.add_variable('stateEstimate.vz', 'FP16')
    # lg_stab.add_variable('gyro.x', 'FP16')
    # lg_stab.add_variable('gyro.y', 'FP16')
    # lg_stab.add_variable('gyro.z', 'FP16')
    
    # lg_stab.add_variable('controller.cmd_thrust', 'FP16')
    # lg_stab.add_variable('controller.cmd_roll', 'FP16')
    # lg_stab.add_variable('controller.cmd_pitch', 'FP16')
    # lg_stab.add_variable('controller.cmd_yaw', 'FP16')



    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        
        simple_log(scf, lg_stab)
