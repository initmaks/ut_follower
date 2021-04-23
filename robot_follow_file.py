import os
import sys
import uuid
import time
import pickle
import logging

import cv2
import numpy as np
import pybullet as p

import pyrealsense2 as rs
from robot_interface_high import RobotInterface # pytype: disable=import-error

from sim import PBsim
from path import Path2D,PathViz
from robot import Robot,RobotViz
from utils import ranged_rotation_angle,angle_diff

T265_SERIAL = "11622110486"

pipeline = rs.pipeline()
config = rs.config()
config.enable_device(T265_SERIAL)
config.enable_stream(rs.stream.pose)
pipeline.start(config)

MAX_DELTA_T = 0.25 # translation
MAX_DELTA_R = 0.4 # rotation

def pbtf2pathtf(tf):
    (cx,cy,_) = tf[0]
    (_,_,ctheta) = p.getEulerFromQuaternion(tf[1])
    return cx,cy,ctheta

def real2pathtf(tf):
    (cx,cy,_) = tf[0]
    (_,_,ctheta) = tf[1]
    return cx,cy,ctheta

def get_control_cmd(current_tf,goal_tf):
    gx,gy,gtheta = goal_tf
    cx,cy,ctheta = current_tf
    if gtheta is None:
        gtheta = np.arctan2(gy-cy,gx-cx)
    dtheta = angle_diff(gtheta,ctheta) # facing towards the next point
    dx = gx-cx
    dy = gy-cy
    r = -ctheta
    ddx = dx * np.cos(r)-dy * np.sin(r)
    ddy = dy * np.cos(r)+dx * np.sin(r)
    return ddx,ddy,dtheta

def robot_move(i, delta_tf):
    dx,dy,dtheta = delta_tf
    highcmd = np.zeros(8, dtype=np.float32)
    highcmd[0] = 2.0 #  mode            // 1 - standing // 2 - walking
    highcmd[1] = dx # cmd.forwardSpeed = highcmd[1];  //  forwardSpeed    [-1,1] = [-0.5,0.8]m/s
    highcmd[2] = dy # cmd.sideSpeed = highcmd[2];     //  sideSpeed       [-1,1] = [-0.3,0.3]m/s
    highcmd[3] = dtheta # cmd.rotateSpeed = highcmd[3];   //  rotateSpeed     [-1,1] = [-50,50]degrees/s
    highcmd[4] = 0.0 # cmd.bodyHeight = highcmd[4];    //  bodyHeight      [-1,1] = [0.3,-0.45]m
    highcmd[5] = 0.0 # cmd.roll = highcmd[5];          //  roll            [-1,1] = [-20,20]degrees
    highcmd[6] = 0.0 # cmd.pitch = highcmd[6];         //  pitch           [-1,1] = [-20,20]degrees
    highcmd[7] = 0.0 # cmd.yaw = highcmd[7];           //  yaw             [-1,1] = [-28,28]degrees

    highcmd[1] = np.clip(highcmd[1],-MAX_DELTA_T,MAX_DELTA_T)
    highcmd[2] = np.clip(highcmd[2],-MAX_DELTA_T,MAX_DELTA_T)
    highcmd[3] = np.clip(highcmd[3],-MAX_DELTA_R,MAX_DELTA_R)

    highcmd[1:] = np.clip(highcmd[1:],-1,1)
    i.send_command(highcmd)
    time.sleep(0.01)

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

        pitch =  -np.arcsin(2.0 * (x*z - w*y));
        roll  =  np.arctan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z);
        yaw   = np.arctan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z);
        pose = (-gz,-gx,gy),(roll, pitch, -yaw)
        last_pose = pose
    return last_pose
        
if __name__=="__main__":

    sampling_frequency = 20
    waypoints_fname = sys.argv[1]
    with open(waypoints_fname,'r') as f:
        trajectory = []
        for i, line in enumerate(f.readlines()):
            if i % sampling_frequency != 0: continue
            x,y,theta = map(float,line.strip().split(','))
            trajectory.append([x,y,theta])
    
    trav_traj_fname = waypoints_fname[:-4] + "_follow_" + str(uuid.uuid4())[:8] + ".txt"

    trajectory = np.array(trajectory)

    # sim = PBsim()

    # robot = Robot()
    # robot_viz = RobotViz(sim)
    # robot_viz.load([0.0,0.0,robot.height,0.0])
    i = RobotInterface()

    path = Path2D(trajectory)
    # path_viz = PathViz(sim)
    # path_viz.load_path(path)

    trajectory_end = False
    # current_tf = robot_viz.current_tf()
    current_tf = get_t265_deltas()

    with open(trav_traj_fname,'w') as f:
        while not trajectory_end:
            try:
                goal_tf = path.get_current_target()
                delta_tf = get_control_cmd(real2pathtf(current_tf),goal_tf)
                # 
                # new_tf = robot.move(current_tf,delta_tf)
                robot_move(i,delta_tf)
                current_tf = get_t265_deltas()

                adx,ady,adt = delta_tf
                (gx,gy,gz),(grol,gpit,gyaw)=current_tf
                if 0: print(f"action {adx:.3f},{ady:.3f},{adt:.3f}")
                if 0: print(f"{gx:.3f},{gy:.3f},{gz:.3f}")
                if 0: print(f"{grol:.3f},{gpit:.3f},{gyaw:.3f}")
                if 0: print(f"state: {gx:.3f},{gy:.3f},{gyaw:.3f}")
                if 0: print(f"action yaw {adt:.4f} yaw: {gyaw:.3f}")
                if 1: print(f"yaw: {gyaw:.3f}")
                if 1: print(f"action {adx:.3f},{ady:.3f},{adt:.3f}")
                if 1: print('=====')
                write_str = f"{gx},{gy},{gyaw}\n"
                f.write(write_str)

                # robot_viz.visualize_base(*new_tf)
                # p.stepSimulation(sim.pClient)
                # current_tf = robot_viz.current_tf()
                is_success,trajectory_end = path.try_step(real2pathtf(current_tf))
                if is_success:
                    lx,ly,ltheta = real2pathtf(current_tf)
                    gx,gy,gtheta = path.get_current_target()
                    print("point reached!")
            except KeyboardInterrupt:
                print('manually terminated!')
                break
    print("TRAJECTORY COMPLETED! \o/ ")
    
    # p.disconnect()
