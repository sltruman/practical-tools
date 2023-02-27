import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation

class Gripper:
    def __init__(self,robot_id,gears):
        self.robot_id = robot_id
        self.gears = gears
        self.joints = dict()
        for j in range(p.getNumJoints(robot_id)):
            ji = p.getJointInfo(robot_id, j)
            id,name,type,lower,upper,max_force,max_velocity = ji[0],ji[1],ji[2],ji[8],ji[9],ji[10],ji[11]
            self.joints[name.decode()] = id,name.decode(),type,lower,upper,max_force,max_velocity
            
        for mimic_name, joint_name, ratio in gears:
            mimic_id = self.joints[mimic_name][0]
            joint_id,name,type,lower,upper,max_force,max_velocity = self.joints[joint_name]
            c = p.createConstraint(robot_id, mimic_id, robot_id, joint_id, jointType=p.JOINT_GEAR,jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
            p.changeConstraint(c, gearRatio=-ratio, maxForce=max_force, erp=1)
        pass

    def update(self,dt):
        pass
        
    
    def do(self,pickup=True):
        for mimic_name, joint_name, ratio in self.gears:
            id,name,type,lower,upper,max_force,max_velocity = self.joints[mimic_name]
            value = 0
            if pickup: value = upper - lower
            p.setJointMotorControl2(self.robot_id, id, p.POSITION_CONTROL, targetPosition=value,force=max_force,maxVelocity=max_velocity)
            id,name,type,lower,upper,max_force,max_velocity = self.joints[joint_name]
            value = 0
            if pickup: value = upper - lower
            p.setJointMotorControl2(self.robot_id, id, p.POSITION_CONTROL, targetPosition=value * ratio,force=max_force,maxVelocity=max_velocity)