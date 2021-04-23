import os
import uuid
import time
import pickle
import logging

import cv2
import numpy as np
import pybullet as p

import pyrealsense2 as rs
from robot_interface_high import RobotInterface # pytype: disable=import-error

T265_SERIAL = "11622110486"

pipeline = rs.pipeline()
config = rs.config()
config.enable_device(T265_SERIAL)
config.enable_stream(rs.stream.pose)
pipeline.start(config)

MAX_DELTA_T = 0.25 # translation
MAX_DELTA_R = 0.4 # rotation

last_pose = [(0.0,0.0,0.0),(0.0,0.0,0.0)]
def get_t265_deltas():
    frames = pipeline.wait_for_frames()
    pose = frames.get_pose_frame()
    if pose:
        # Print some of the pose data to the terminal
        data = pose.get_pose_data()

        gx = data.translation.x
        gy = data.translation.y
        gz = data.translation.z

        w = data.rotation.w
        x = -data.rotation.z
        y = data.rotation.x
        z = -data.rotation.y

        pitch =  -np.arcsin(2.0 * (x*z - w*y))
        roll  =  np.arctan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z)
        yaw   = np.arctan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z)
        pose = (-gz,-gx,gy),(roll, pitch, -yaw)
        last_pose = pose
    return last_pose
        
if __name__=="__main__":
    i = RobotInterface()

    counter = 0
    run_fname = "init_attempt" + str(uuid.uuid4())[:8] + ".txt"
    with open(run_fname,'w') as f:
        while True:
            try:
                current_tf = get_t265_deltas()
                (gx,gy,gz),(grol,gpit,gyaw)=current_tf

                write_str = f"{gx},{gy},{gyaw}\n"
                f.write(write_str)
                if counter%10==0: print(f"{gx:.3f},{gy:.3f},{gyaw:.3f}")
                counter+=1
            except KeyboardInterrupt:
                print('All done')
                break

