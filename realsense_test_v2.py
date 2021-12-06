import cv2
import numpy as np
import pyrealsense2 as rs
from icecream import ic

def runRealSenseTracking():
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

  while True:
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
        depth_intrinsics = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        
				# Image -> Camera
        (x_, y_, z_) = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [x, y], dist)
        print("Move X: %.2f, Y: %.2f" % (-x_, y_))

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

      if cv2.waitKey(10) & 0xFF == ord('q'):
          break

  pipe.stop()
  cv2.destroyAllWindows()
  return

if __name__ == "__main__":
  print(cv2.__version__)
  runRealSenseTracking()