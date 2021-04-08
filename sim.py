import time

import numpy as np
import pybullet as p
import pybullet_data

# visualization of robot and path in pybullet + control
# no physics

class PBsim:
    def __init__(self) -> None:
        self.pClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,0,physicsClientId=self.pClient)
        floor = p.loadURDF("plane.urdf",physicsClientId=self.pClient)
        
    def load_sphere(self,loc,fixed_base=True):
        return p.loadURDF(
            "sphere2.urdf",
            basePosition=loc,
            useFixedBase=fixed_base,
            globalScaling=0.1,
            physicsClientId=self.pClient
        )

    def load_cube(self,loc,orn,fixed_base=True):
        return p.loadURDF(
            "cube.urdf",
            basePosition=loc,
            baseOrientation=p.getQuaternionFromEuler(orn),
            useFixedBase=fixed_base,
            globalScaling=0.1,
            physicsClientId=self.pClient)
    
    def __del__(self):
        p.disconnect(self.pClient)