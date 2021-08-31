import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
import time

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 63) # acceleration
config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)  # gyroscope
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)  # depth

# Start streaming
profile = pipeline.start(config)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: ", depth_scale)

try:
    while True:
        frames = pipeline.wait_for_frames(10000)
        depth_frame = frames.get_depth_frame()
        accel_frame = frames.first_or_default(rs.stream.accel, rs.format.motion_xyz32f)
        gyro_frame = frames.first_or_default(rs.stream.gyro, rs.format.motion_xyz32f)
        print("\n\taccel = {}, \n\tgyro = {}".format(str(accel_frame.as_motion_frame().get_motion_data()), str(gyro_frame.as_motion_frame().get_motion_data())))
        time.sleep(0.5)
        depth = frames.get_depth_frame()
        for y in range(480):
            for x in range(640):
                dist = depth.get_distance(x,y)
                if dist > 0.1 and dist < 0.5:
                    print("\n\tdist = {}".format(str(dist)))
finally:
    # Stop streaming
    pipeline.stop()

