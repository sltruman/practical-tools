import numpy as np
from scipy.spatial.transform import Rotation
import pybullet as p

class Suction:
    def __init__(self,robot_id):
        self.robot_id = robot_id
        self.action = None
        pass

    def update(self,dt):
        if self.action:
            f,args = self.action
            f(args)
        pass

    def do(self,pickup=True):
        if pickup:
            num_joints = p.getNumJoints(self.robot_id)
            ee_index = num_joints-1
            ee_pos,ee_orn,_,_,_,_ = p.getLinkState(self.robot_id,ee_index)
            radius = 0.01
            axis_z = Rotation.from_quat(ee_orn).apply(np.array([0,0,-radius])) + ee_pos
            rayInfo = p.rayTest(ee_pos, axis_z)
            p.addUserDebugLine(ee_pos,axis_z,[0,1,0],4,lifeTime=1)

            if rayInfo: 
                o_id,linkindex,fraction,o_pos,norm = rayInfo[0]
                
                def task(o_id):
                    o_pos = p.getBasePositionAndOrientation(o_id)[0]
                    ee_pos,_,_,_,_,_ = p.getLinkState(self.robot_id,ee_index)
                    ee_pos,o_pos = np.array(ee_pos),np.array(o_pos)
                    direction = ee_pos - o_pos
                    direction = direction / np.linalg.norm(direction)
                    p.applyExternalForce(o_id,-1,direction * 9.81 ,o_pos,p.WORLD_FRAME)
                    pass

                if o_id != -1:
                    self.action = (task,(o_id))
        else:
            self.action = None
        pass