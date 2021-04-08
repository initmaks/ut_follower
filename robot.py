import numpy as np
import pybullet as p
from utils import ranged_rotation_angle

class Robot:
    "simple robot class that generates the motion transforms"
    def __init__(self):
        self.max_speed = 0.1
        self.max_rotation = 0.3
        self.height = 0.3

    def move(self,current_tf,tf):
        dx,dy,dtheta=tf
        dx = np.clip(dx,-self.max_speed,self.max_speed)
        dy = np.clip(dy,-self.max_speed,self.max_speed)
        dtheta = np.clip(dtheta,-self.max_rotation,self.max_rotation)

        new_xyz,rot_quat = current_tf
        new_rpy = np.array(p.getEulerFromQuaternion(rot_quat))

        new_rpy[2] += dtheta
        new_rpy[2] = ranged_rotation_angle(new_rpy[2])
        facing_ang = new_rpy[2]

        d_ang = np.arctan2(dy,dx)
        speed = min(np.sqrt(dx**2+dy**2),self.max_speed)
        ndx = speed * np.cos(facing_ang+d_ang)
        ndy = speed * np.sin(facing_ang+d_ang)

        new_xyz += np.array([ndx, ndy, 0.0])
        new_xyz[2] = self.height

        return new_xyz,new_rpy

class RobotViz:
    def __init__(self,sim):
        self.sim = sim
        self.pClient = sim.pClient
        self.facing_line_id = None

    def load(self,tf):
        x,y,z,theta = tf
        loc = [x,y,z]
        rot = [0,0,theta]
        self.objID = self.sim.load_cube(loc,rot,True)

    def current_tf(self):
        return p.getBasePositionAndOrientation(self.objID,physicsClientId=self.sim.pClient)

    def visualize_base(self,xyz,rpy):
        p.resetBasePositionAndOrientation(
            self.objID,
            posObj=xyz,
            ornObj=p.getQuaternionFromEuler(rpy),
            physicsClientId=self.pClient
        )
        p.resetDebugVisualizerCamera(
            cameraDistance=1,
            cameraYaw=(rpy[2]/np.pi*180)-90,
            cameraPitch=-30,
            cameraTargetPosition=xyz,
            physicsClientId=self.pClient
        )

