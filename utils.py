import numpy as np

def lin_dist(point1,point2):
    if type(point1) is not np.ndarray: point1 = np.array(point1)
    if type(point2) is not np.ndarray: point2 = np.array(point2)
    return np.linalg.norm(point1-point2)

def rot_dist(a,b):
    raw_diff = np.abs(a-b)
    mod_diff = raw_diff % (2*np.pi)
    return np.pi - np.abs(mod_diff - np.pi)

def ranged_rotation_angle(r):
    # makes sure the r is always between -pi and pi
    r %= np.pi*2
    if r < 0.0: r+= np.pi*2
    return r

