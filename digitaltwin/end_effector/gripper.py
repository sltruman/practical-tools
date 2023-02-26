import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation

class Gripper:
    def __init__(self,robot_id,gears,open=0,close=0.62):
        for mimic_name, joint_name, ratio in gears:
            mimic_id = None
            joint_id = None
            for j in p.getNumJoints(self.robot_id):
                ji = p.getJointInfo(self.robot_id, j)
                id,name,type = ji[0],ji[1],ji[2]
                if mimic_name == name: mimic_id = id
                if joint_name == name: joint_id = id
            
            if not (mimic_id and joint_id): continue
            c = p.createConstraint(self.robot_id, mimic_id, self.robot_id, id, jointType=p.JOINT_GEAR,jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
            p.changeConstraint(c, gearRatio=-ratio, maxForce=100, erp=1)
        pass
    
    def do(self):
        close = 0.6273989381446866
        open = 0
        p.setJointMotorControl2(self.robot_id, 
                                self.mimic_parent_id, 
                                p.POSITION_CONTROL, 
                                targetPosition=open_angle,
                                force=self.joints[self.mimic_parent_id]['max_force'], 
                                maxVelocity=self.joints[self.mimic_parent_id]['max_velocity'])
        for joint_id, multiplier in self.mimic_child_multiplier.items():
            p.setJointMotorControl2(self.robot_id, 
                                    joint_id,
                                    p.POSITION_CONTROL,
                                    targetPosition=open_angle * multiplier,
                                    force=self.joints[joint_id]['max_force'],
                                    maxVelocity=self.joints[joint_id]['max_velocity'])
        pass 