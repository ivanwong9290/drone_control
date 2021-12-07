import cv2
import numpy as np
import pyrealsense2 as rs
import logging
import time
# import keyboard
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.commander import Commander



def runRealSenseTracking():
  # Instantiate Realsense stuff
  pipe = rs.pipeline()
  config = rs.config()
  config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
  config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 60)
  profile = pipe.start(config)

  depth_sensor = profile.get_device().first_depth_sensor()
  depth_sensor.set_option(rs.option.noise_filtering, 6)

  frameset = pipe.wait_for_frames()
  color_frame = frameset.get_color_frame()
  color_init = np.asanyarray(color_frame.get_data())

  font                   = cv2.FONT_HERSHEY_SIMPLEX
  bottomLeftCornerOfText = (10,500)
  fontScale              = 1
  fontColor              = (255,255,255)
  lineType               = 2

  # URI to the Crazyflie to connect to
  uri = 'radio://0/80/2M/E7E7E7E7E7'


# Only output errors from the logging framework
  logging.basicConfig(level=logging.ERROR)

  # Initialize the low-level drivers (don't list the debug drivers)
  cflib.crtp.init_drivers(enable_debug_driver=False)

  # Defining logging configs
  lg_stab = LogConfig(name='state', period_in_ms=100)
  lg_stab.add_variable('stateEstimate.x', 'FP16')
  lg_stab.add_variable('stateEstimate.y', 'FP16')
  lg_stab.add_variable('stateEstimate.z', 'FP16')
  lg_stab.add_variable('stateEstimate.roll', 'FP16')
  lg_stab.add_variable('stateEstimate.pitch', 'FP16')
  lg_stab.add_variable('stateEstimate.yaw', 'FP16')
  lg_stab.add_variable('stateEstimate.vx', 'FP16')
  lg_stab.add_variable('stateEstimate.vy', 'FP16')
  lg_stab.add_variable('stateEstimate.vz', 'FP16')
  lg_stab.add_variable('gyro.x', 'FP16')
  lg_stab.add_variable('gyro.y', 'FP16')
  lg_stab.add_variable('gyro.z', 'FP16')
  
  endTime_hover = time.time() + 10
  endTime_realsense = endTime_hover + 5

  yes = False
  # main loop
  while True:
      # if (yes == False):
      #   with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
      #     simple_log(scf, lg_stab)
      #   yes = True
      # Updates new frame
      frameset = pipe.wait_for_frames()
      color_frame = frameset.get_color_frame()
      color = np.asanyarray(color_frame.get_data())
      depth_frame = frameset.get_depth_frame()

      res = color.copy()

      hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
      l = np.array([160, 50, 50])
      u = np.array([180, 255, 255])

      mask = cv2.inRange(hsv, l, u)
      color = cv2.bitwise_and(color, color, mask=mask)

      colorizer = rs.colorizer()
      colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())

      align = rs.align(rs.stream.color)
      frameset = align.process(frameset)

      aligned_depth_frame = frameset.get_depth_frame()
      colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())

      # Subtraction image
      diff = cv2.absdiff(color_init, color)
      gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
      gray = cv2.GaussianBlur(gray, (11, 11), 0.5)
      _, binary = cv2.threshold(gray, 20, 255, cv2.THRESH_BINARY)
      binary = cv2.dilate(binary, np.ones((3, 3), np.uint8), iterations=3) # 15
      c, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      color_init = color

      # cv2.drawContours(res, c, -1, (255, 0, 0), 4)
      
      depth = np.asanyarray(aligned_depth_frame.get_data())
      
      if len(c) > 0:
        contour = max(c, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(contour)
        M = cv2.moments(contour)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        cv2.circle(res, (int(x), int(y)), int(radius), (0, 255, 0), 3)
        cv2.circle(res, center, 5, (0, 255, 0), -1)

        dist = aligned_depth_frame.get_distance(int(x), int(y))
        if dist == 0:
          continue
        depth_intrinsics = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        
				# Image -> Camera
        (x_, y_, z_) = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [x, y], dist)
        print("Move X: %.2f, Y: %.2f" % (-x_, y_))
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
          cf = scf.cf
          if time.time() <= endTime_hover:
            # cf.commander.send_hover_setpoint(0,0,0,0.3)
            # Move (x, z, y = change in height, yaw)
            cf.commander.send_position_setpoint(-x_, 0, y_, 0)

				# Print x, y
        text = "x: " + str("{0:.2f}").format(x_) + "m, y: " + str("{0:.2f}").format(-y_) + "m"
        cv2.putText(res,
                    text, 
                    (int(x), int(y)-30), 
                    font, fontScale, 
                    fontColor, 
                    lineType)

				# Print z
        text = "Depth: " + str("{0:.2f}").format(z_) + "m"
        cv2.putText(res,
                    text,
                    (int(x - radius), int(y + radius)),
                    font,
                    fontScale,
                    fontColor,
                    lineType)

			# Put reference point on screen
      cv2.circle(res, (480, 270), 5, (0, 0, 255), -1)
      text = "Ref"
      cv2.putText(res,
                  text,
                  (480, 250),
                  font,
                  fontScale,
                  fontColor,
                  lineType)

      cv2.namedWindow('Normal', cv2.WINDOW_AUTOSIZE)
      cv2.imshow('Normal', res)
      # cv2.namedWindow('Depth', cv2.WINDOW_AUTOSIZE)
      # cv2.imshow('Depth', colorized_depth)
      cv2.namedWindow('Mask', cv2.WINDOW_AUTOSIZE)
      cv2.imshow('Mask', color)
      # cv2.namedWindow('Binary', cv2.WINDOW_AUTOSIZE)
      # cv2.imshow('Binary', binary)
      cv2.waitKey(20)

      if (cv2.waitKey(10) & 0xFF == ord('q')) or time.time() > endTime_realsense:
          break

  pipe.stop()
  cv2.destroyAllWindows()
  return

def simple_log(scf, lg_stab):
    
    with SyncLogger(scf, lg_stab) as logger:
        endTime = time.time() + 10
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

            print('[%d]: %s' % (timestamp, data))

            if time.time() > endTime:
                print("Exiting Logging after 10 sec")
                break

        stateData = np.asarray(stateData)
        np.save("stateData.npy",stateData)

        print("Saved Data")

        return
if __name__ == "__main__":
  print(cv2.__version__)

  runRealSenseTracking()