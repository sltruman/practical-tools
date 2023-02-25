import numpy as np
from scipy.spatial.transform import Rotation
import pybullet as p

class Suction:
    def __init__(self,base):
        self.base = base

        # num_joints = p.getNumJoints(self.robot_id)
        # pos,orn,_,_,_,_ = p.getLinkState(self.robot_id,num_joints-1)
        # self.id = p.loadURDF(base, pos, orn)

        # self.joint_end_effector = p.createConstraint(self.robot_id,num_joints-1,self.id,-1,p.JOINT_FIXED,[0,0,1],[0,0,0],[0,0,0])

        # min,max = p.getAABB(self.id)
        # min,max = np.array(min),np.array(max)
        # center = (max - min) / 2 
        # length = np.max(center)
        
        # num_joints = p.getNumJoints(self.robot_id)
        # pos,orn,_,_,_,_ = p.getLinkState(self.robot_id,num_joints-1)
        # axis_x = Rotation.from_quat(orn).apply(np.array([0.05,0,0])) + pos
        # axis_y = Rotation.from_quat(orn).apply(np.array([0,0.05,0])) + pos
        # axis_z = Rotation.from_quat(orn).apply(np.array([0,0,0.05])) + pos
        # p.addUserDebugLine(pos,axis_x,[1,0,0],2)
        # p.addUserDebugLine(pos,axis_y,[0,1,0],2)
        # p.addUserDebugLine(pos,axis_z,[0,0,1],2)

    def __del__(self):
        # if 'id' in vars(self): p.removeBody(self.id)
        pass

    def do(self, open):
        if open:
            print('do open')
            # p.ray()

            # closest_points = p.getClosestPoints(self.robot_id,o_id,0.02,7)
            # if closest_points:
            #     p.applyExternalForce(o_id,-1,direction_norm * 20 * obj_mass * 9.81 ,o_pos,p.WORLD_FRAME)
            
            # return
        else:
            print('do close')