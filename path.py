import numpy as np
import pybullet as p

from utils import *

class Path2D:
    def __init__(self,points:np.array) -> None:
        assert points.shape[1] == 3, "positions array format [x,y,theta]"
        self.original_path = points
        self.current_point_idx = 0
        self.n_points = self.original_path.shape[0]
        self.line_id = None
    
    def get_current_target(self):
        x,y,theta = self.original_path[self.current_point_idx]
        return x,y,theta
    
    def is_reached(self,current,target,dist_tol=0.1,rot_tol=0.1): # TODO check + tune tollerances!
        cx,cy,ctheta = current
        x,y,theta = target
        dist_reached = lin_dist((cx,cy),(x,y)) < dist_tol
        if theta is None:
            return dist_reached
        else:
            rot_reached = np.abs(angle_diff(ctheta,theta)) < rot_tol
            return dist_reached and rot_reached

    def try_step(self,current_tf):
        target_tf = self.get_current_target()
        is_success = self.is_reached(current_tf,target_tf)
        trajectory_end = False
        if is_success:
            self.current_point_idx += 1
            if self.current_point_idx >= self.n_points:
                self.current_point_idx = self.n_points-1
                trajectory_end = True
        return is_success, trajectory_end

class PathViz:
    def __init__(self,sim):
        self.sim = sim
        self.pClient = sim.pClient
        self.height = 0.2
        self.line_id = None
    
    def load_path(self,path):
        self.current_path = path
        self.checkpoint_obj_ids = []
        for x,y,theta in path.original_path:
            loc = [x,y,self.height]
            checkpoint_obj_id = self.sim.load_sphere(loc)
            ex = x+0.4 * np.cos(theta)
            ey = y+0.4 * np.sin(theta)
            end = [ex,ey,loc[2]]
            facing_line_id = 0
            # facing_line_id = p.addUserDebugLine(
            #     lineFromXYZ=loc,
            #     lineToXYZ=end,
            #     lineColorRGB=[255,0,0],
            #     lineWidth=3.0,
            #     lifeTime=0.2,
            #     physicsClientId=self.pClient)
            self.checkpoint_obj_ids.append((checkpoint_obj_id,facing_line_id))
    
    def draw_line(self,fx,fy,tx,ty):
        from_p = [fx,fy,self.height]
        to_p = [tx,ty,self.height]
        if self.line_id:
            self.line_id = p.addUserDebugLine(
                lineFromXYZ=from_p,
                lineToXYZ=to_p,
                lineColorRGB=[255,0,0],
                lineWidth=15.0,
                lifeTime=0,
                replaceItemUniqueId=self.line_id,
                physicsClientId=self.pClient)
        else:
            self.line_id = p.addUserDebugLine(
                lineFromXYZ=from_p,
                lineToXYZ=to_p,
                lineColorRGB=[255,0,0],
                lineWidth=15.0,
                lifeTime=0,
                physicsClientId=self.pClient)
