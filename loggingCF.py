# import logging
# from os import wait
# import time
# import numpy as np
# import cflib.crtp
# from cflib.crazyflie import Crazyflie
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.crazyflie.log import LogConfig
# from cflib.crazyflie.syncLogger import SyncLogger
# from cflib.crazyflie.commander import Commander
# from camera import camera
# import matplotlib.pyplot as plt

# # URI to the Crazyflie to connect to
# uri = 'radio://0/80/2M/E7E7E7E7E7'


# # Only output errors from the logging framework
# logging.basicConfig(level=logging.ERROR)



# def simple_log(scf, logconf):
#     c1 = camera()
#     c1.config()
    
#     x_init, z_init, y_init = c1.avgCoords()
#     z_init = -z_init
#     hoverHeight = 0.2

#     x_ref_drone, y_ref_drone, z_ref_drone = 0, 0, hoverHeight
    
#     x_ref, y_ref, z_ref = x_init, y_init, z_init + hoverHeight
    
#     print('Intial Position in Camera Frame: ', x_init, ' ', y_init, ' ', z_init)
#     print('Reference Position  in Camera Frame: ', x_ref, ' ', y_ref, ' ', z_ref)

#     print('Desired Position: ', x_ref - x_init, ' ', y_ref - y_init, ' ', z_ref - z_init)
    
#     time.sleep(5)
    

#     dronePos_camera = []

#     with SyncLogger(scf, lg_stab) as logger:
        
#         startTime = time.time() 
#         endTime = startTime + 10

#         print(startTime, ' ', endTime)

#         cf = scf.cf
#         stateData = []
#         i = 1

#         for log_entry in logger:
#             print(i)
#             x_curr, z_curr, y_curr = c1.avgCoords()
#             z_curr = -z_curr

#             x_drone = x_curr - x_init
#             y_drone = y_curr - y_init
#             z_drone = z_curr - z_init

#             ex_drone = x_drone - x_ref_drone
#             ey_drone = y_drone - y_ref_drone
#             ez_drone = z_drone - z_ref_drone
            
#             # cf.commander.send_position_setpoint(0, 0, 0.3, 0)
#             cf.commander.send_hover_setpoint(0,0,0,0.4)
#             # if(time.time() >= startTime + 1): cf.commander.send_hover_setpoint(0,0,0,hoverHeight)
#             # else: cf.commander.send_position_setpoint(-ex_drone, -ey_drone, -ez_drone+hoverHeight, 0)
            
#             print('Error Position: ', ex_drone, ' ', ey_drone, ' ', ez_drone)
#             dronePos_camera.append([x_drone, y_drone, z_drone])
#             # if i <= 10: # initially get the drone to take off
#             #     cf.commander.send_hover_setpoint(0,0,0,i/25) 
#             #     time.sleep(0.1)
#             # else: # now loop to keep drone at certain height
#             #     cf.commander.send_hover_setpoint(0,0,0,0.4)
#             #     time.sleep(0.1)
#             # i+=1

#             timestamp = log_entry[0]
#             data = log_entry[1]
#             logconf_name = log_entry[2]
#             stateData.append(data)

#             print('[%d]: %s' % (timestamp, data))

#             i+=1
#             if time.time() > endTime:
#                 print("Exiting Logging after 10 sec")
#                 break
        
#         dronePos_camera = np.asarray(dronePos_camera)
#         print(dronePos_camera.shape)
        
#         plt.plot(dronePos_camera)
#         plt.legend(['X', 'Y', 'Z'])
#         plt.show()
        
#         stateData = np.asarray(stateData)
#         np.save("stateData.npy",stateData)

#         print("Saved Data")

#         return


# if __name__ == '__main__':

#     # Initialize the low-level drivers (don't list the debug drivers)
#     cflib.crtp.init_drivers(enable_debug_driver=False)


#     # Defining logging configs
#     lg_stab = LogConfig(name='state', period_in_ms=10)
#     # lg_stab.add_variable('stateEstimate.x', 'FP16')
#     # lg_stab.add_variable('stateEstimate.y', 'FP16')
#     # lg_stab.add_variable('stateEstimate.z', 'FP16')
#     # lg_stab.add_variable('stateEstimate.roll', 'FP16')
#     # lg_stab.add_variable('stateEstimate.pitch', 'FP16')
#     # lg_stab.add_variable('stateEstimate.yaw', 'FP16')
#     # lg_stab.add_variable('stateEstimate.vx', 'FP16')
#     # lg_stab.add_variable('stateEstimate.vy', 'FP16')
#     # lg_stab.add_variable('stateEstimate.vz', 'FP16')
#     # lg_stab.add_variable('gyro.x', 'FP16')
#     # lg_stab.add_variable('gyro.y', 'FP16')
#     # lg_stab.add_variable('gyro.z', 'FP16')
#     lg_stab.add_variable('controller.cmd_thrust', 'FP16')
#     lg_stab.add_variable('controller.cmd_roll', 'FP16')
#     lg_stab.add_variable('controller.cmd_pitch', 'FP16')
#     lg_stab.add_variable('controller.cmd_yaw', 'FP16')



#     with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        
#         simple_log(scf, lg_stab)

import logging
import time
# import keyboard
import numpy as np
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



def simple_log(scf, logconf):

    with SyncLogger(scf, lg_stab) as logger:
        endTime = time.time() + 30
        cf = scf.cf
        stateData = []
        i = 0

        for log_entry in logger:

            cf.commander.send_hover_setpoint(0,0,0,0.3)

            # if i <= 10: # initially get the drone to take off
            #     cf.commander.send_hover_setpoint(0,0,0,i/25) 
            #     time.sleep(0.1)
            # else: # now loop to keep drone at certain height
            #     cf.commander.send_hover_setpoint(0,0,0,0.4)
            #     time.sleep(0.1)
            # i+=1

            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]
            stateData.append(data)

            # print('[%d]: %s' % (timestamp, data))

            if time.time() > endTime:
                print("Exiting Logging after 10 sec")
                break

        stateData = np.asarray(stateData)
        np.save("stateData.npy",stateData)

        print("Saved Data")

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
    # lg_stab.add_variable('stabilizer.thrust', 'FP16')
    # lg_stab.add_variable('controller.cmd_thrust', 'FP16')
    # lg_stab.add_variable('controller.cmd_roll', 'FP16')
    # lg_stab.add_variable('controller.cmd_pitch', 'FP16')
    # lg_stab.add_variable('controller.cmd_yaw', 'FP16')



    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        
        simple_log(scf, lg_stab)