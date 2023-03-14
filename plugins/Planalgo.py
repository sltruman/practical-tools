import numpy as np
from scipy.spatial.transform import Rotation

result = None

def plan(origin,target,*args):
    objs,robot_pos,robot_rot,end_effector_pos,end_effector_rot,rotbot_joint_poses = args
    ee_pos,ee_rot = origin
    o_pos,o_rot = target
    o_pos = np.array(o_pos)

    route_poses = list()
    route_poses.append([-1.57,0.0,-0.3925,-0.785,1.57, 0, 0, 0, 0, 0, 0, 0])
    route_poses.append([-1.57,0.0,-0.3925,-0.785,1.4, 0, 0, 0, 0, 0, 0, 0])
    route_poses.append([-1.57,0.0,-0.3925,-0.785,1.3, 0, 0, 0, 0, 0, 0, 0])
    route_poses.append([-1.57,0.0,-0.3925,-0.785,1.2, 0, 0, 0, 0, 0, 0, 0])
    route_poses.append([-1.57,0.0,-0.3925,-0.785,1.1, 0, 0, 0, 0, 0, 0, 0])
    route_poses.append([-1.57,0.0,-0.3925,-0.785,1.0, 0, 0, 0, 0, 0, 0, 0])
    route_poses.append([-1.57,0.0,-0.3925,-0.785,0.9, 0, 0, 0, 0, 0, 0, 0])
    return route_poses 