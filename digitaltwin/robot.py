import pybullet as p
from .active_obj import ActiveObject
import numpy as np
from scipy.spatial.transform import Rotation
from .end_effector import Gripper,Suction

class Robot(ActiveObject):
    def __init__(self,scene,**kwargs):
        self.joint_poses = self.reset_joint_poses = kwargs['reset_joint_poses']
        self.joint_damping = kwargs['joint_damping']
        super().__init__(scene, **kwargs)

        self.end_effector_kind = kwargs['end_effector_kind']
        self.set_end_effector(kwargs['end_effector'])
        
        self.end_effector_gear_joints = kwargs['end_effector_gear_joints']
        self.pick_pose = kwargs['pick_pose']
        pass

    def properties(self):
        info = super().properties()
        info.update(dict(kind='Robot',end_effector=self.end_effector.base))
        return info

    def set_base(self,base):
        if 'end_effector_id' in vars(self): p.removeBody(self.end_effector_id)
        super().set_base(base)

        # if 'end_effector' in vars(self): self.set_end_effector(self.end_effector)
        return self.id

    def set_end_effector(self,end_effector_base):
        if 'end_effector' in vars(self): del self.end_effector
        
        if self.end_effector_kind == 'Gripper':
            self.end_effector = Gripper(end_effector_base,self.id,self.end_effector_gear_joints)
        else:
            self.end_effector = Suction(end_effector_base,self.id)
        return 0

    def set_pick_pose(self):
        pass

    def signal_pick_plan(self,camera,objs):
        c_pos,c_rot,fov,focal,d_piexls,pixels = camera

        def key(o):
            o_pos,o_rot = o
            e_pos,e_orn,_,_,_,_ = p.getLinkState(self.id,p.getNumJoints(self.id)-1)
            o_pos,e_pos = np.array(o_pos),np.array(e_pos)

            distance = np.linalg.norm(e_pos - o_pos)
            return o_pos[2] * 10 + distance

        o = max(objs,key=key)

        o_pos,o_rot = o
        pick_pos,pick_rot = self.pick_pose['pos'],self.pick_pose['rot']
        o_pos,pick_pos = np.array(o_pos),np.array(pick_pos)

        route = list()
        route.append((o_pos + pick_pos,o_rot,pick_rot))

        points = list()
        num_joints = p.getNumJoints(self.id)
        ee_index = num_joints-1

        for o_pos,o_rot,pick_rot in route:
            ee_pos,ee_orn,_,_,_,_ = p.getLinkState(self.id,ee_index)
            o_pos,ee_pos = np.array(o_pos),np.array(ee_pos)

            while np.linalg.norm(o_pos - ee_pos) != 0:
                direction = o_pos - ee_pos
                length = np.linalg.norm(direction)
                if length < (self.scene.timestep):  
                    increment_direction = direction
                else: 
                    increment_direction = direction / length * (self.scene.timestep)
                ee_pos += increment_direction

                p.addUserDebugPoints([ee_pos],[[1,0,0]],2)
                points.append((np.array(ee_pos),np.array(pick_rot)))

        def output(): self.result = None,points,
        self.actions.append((output,()))

    def signal_move(self,points:list):
        def task(ee_pos,ee_rot):
            ee_orn = p.getQuaternionFromEuler(ee_rot)
            num_joints = p.getNumJoints(self.id)
            ee_index = num_joints-1
            poses = p.calculateInverseKinematics(self.id, ee_index, ee_pos, ee_orn)
            i = 0
            for j in range(num_joints):
                ji = p.getJointInfo(self.id, j)
                jointName,jointType = ji[1],ji[2]
                if (jointType == p.JOINT_REVOLUTE):
                    p.setJointMotorControl2(self.id, j, p.POSITION_CONTROL, poses[i])
                    i+=1

            pos,orn,_,_,_,_ = p.getLinkState(self.id,num_joints-1)
            axis_x = Rotation.from_quat(orn).apply(np.array([0.05,0,0])) + pos
            axis_y = Rotation.from_quat(orn).apply(np.array([0,0.05,0])) + pos
            axis_z = Rotation.from_quat(orn).apply(np.array([0,0,0.05])) + pos
            p.addUserDebugLine(pos,axis_x,[1,0,0],2)
            p.addUserDebugLine(pos,axis_y,[0,1,0],2)
            p.addUserDebugLine(pos,axis_z,[0,0,1],2)
        
        for ee_pos,ee_rot in points:
            self.actions.append((task,(ee_pos,ee_rot)))
        
        def output(): self.result = None,
        self.actions.append((output,()))
        pass

    def signal_place_plan(self,objs):
        pass

    def signal_do(self):
        def output(): self.result = None,
        self.actions.append((output,))
        pass

    def signal_place(self,origin,pick_rot,target,rot):
        def output(): self.result = None,
        self.actions.append((output,))
        pass
