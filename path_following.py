import time

import numpy as np
import pybullet as p

from sim import PBsim
from path import Path2D,PathViz
from robot import Robot,RobotViz
from utils import ranged_rotation_angle,rot_dist

trajectory = [
    [0.0,0.0,0.0],
    [4.0,0.0,0.0],
    [4.0,5.0,0.0],
    [0.0,10.0,0.0]
]

trajectory = np.array(trajectory)

def pbtf2pathtf(tf):
    (cx,cy,_) = tf[0]
    (_,_,ctheta) = p.getEulerFromQuaternion(tf[1])
    return cx,cy,ctheta

def get_control_cmd(current_tf,goal_tf):
    gx,gy,_ = goal_tf
    cx,cy,ctheta = current_tf
    gtheta = np.arctan2(gy-cy,gx-cx)
    dtheta = gtheta-ctheta # facing towards the next point
    if np.abs(dtheta) > 0.01:
        return 0.0,0.0,dtheta
    else:
        dx = gx-cx
        dy = gy-cy
        r = -ctheta
        ddx = dx * np.cos(r)-dy * np.sin(r)
        ddy = dy * np.cos(r)+dx * np.sin(r)
        return ddx,ddy,dtheta*0.2

if __name__=="__main__":
    sim = PBsim()

    robot = Robot()
    robot_viz = RobotViz(sim)
    robot_viz.load([0.0,0.0,robot.height,0.0])

    path = Path2D(trajectory)
    path_viz = PathViz(sim)
    path_viz.load_path(path)

    trajectory_end = False
    current_tf = robot_viz.current_tf()
    while not trajectory_end:
        goal_tf = path.get_current_target()
        delta_tf = get_control_cmd(pbtf2pathtf(current_tf),goal_tf)
        new_tf = robot.move(current_tf,delta_tf)
        robot_viz.visualize_base(*new_tf)
        p.stepSimulation(sim.pClient)
        current_tf = robot_viz.current_tf()
        is_success,trajectory_end = path.try_step(pbtf2pathtf(current_tf))
        if is_success:
            lx,ly,ltheta = pbtf2pathtf(current_tf)
            gx,gy,gtheta = path.get_current_target()
            path_viz.draw_line(
                fx=lx,
                fy=ly,
                tx=gx,
                ty=gy
                )
            # TODO add visualization that target is reached
        # time.sleep(1/30)
        time.sleep(1/10)
    
    # TRAJECTORY is OVER - rotate robot
    for i in range(20):
        current_tf = robot_viz.current_tf()
        delta_tf = 0.0,0.0,0.5
        new_tf = robot.move(current_tf,delta_tf)
        robot_viz.visualize_base(*new_tf)
        p.stepSimulation(sim.pClient)
        time.sleep(1/10)

    p.disconnect()