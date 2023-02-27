import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation
import xml.dom.minidom as xml
import os 

from .active_obj import ActiveObject
from .end_effector import Gripper,Suction


class Robot(ActiveObject):
    def __init__(self,scene,**kwargs):
        self.reset_joint_poses = kwargs['reset_joint_poses']
        self.joint_damping = kwargs['joint_damping']
        self.pick_pose = kwargs['pick_pose']
        self.end_effector = kwargs['end_effector']
        super().__init__(scene, **kwargs)
        self.pickup = False
        pass

    def properties(self):
        info = super().properties()
        info.update(dict(kind='Robot',end_effector=self.end_effector))
        return info
    
    def update(self,dt):
        super().update(dt)
        self.end_effector_obj.update(dt)
        pass

    def set_base(self,base):
        self.base = base
        
        import tempfile
        f = tempfile.NamedTemporaryFile("w")
        base_temp = f.name

        ee_kind = ""
        mimic_name = ''
        gears = []
        with xml.parse(self.base) as doc_robot:
            node_robot = doc_robot.getElementsByTagName('robot')[0]
            
            for mesh in node_robot.getElementsByTagName('mesh'):
                filename = mesh.getAttribute('filename')
                mesh.setAttribute('filename',os.path.join(os.getcwd(),os.path.dirname(base),filename))

            if 'end_effector' in vars(self):
                link_ee = node_robot.getElementsByTagName('link')[-1]
                link_ee_name = link_ee.getAttribute('name')
                with xml.parse(self.end_effector) as doc_ee:
                    node_ee = doc_ee.getElementsByTagName('robot')[0]
                    ee_kind = node_ee.getAttribute("kind")
                    for mesh in node_ee.getElementsByTagName('mesh'):
                        filename = mesh.getAttribute('filename')
                        mesh.setAttribute('filename',os.path.join(os.getcwd(),os.path.dirname(self.end_effector),filename))
                    for i,link in enumerate(node_ee.getElementsByTagName('link')):
                        if i == 0: link.setAttribute('name',link_ee.getAttribute('name'))
                        node_robot.appendChild(link)
                    for i,joint in enumerate(node_ee.getElementsByTagName('joint')): 
                        if i == 0: joint.getElementsByTagName('parent')[0].setAttribute('link',link_ee_name)
                        node_robot.appendChild(joint)
                        node_mimics = joint.getElementsByTagName('mimic')
                        if not node_mimics: continue
                        node_mimic = node_mimics[0]
                        mimic_name = node_mimic.getAttribute('joint')
                        joint_name = joint.getAttribute('name')
                        multiplier = int(node_mimic.getAttribute('multiplier'))
                        offset = node_mimic.getAttribute('offset')
                        gears.append((mimic_name,joint_name,multiplier))

                node_robot.removeChild(link_ee).unlink()
            f.write(node_robot.toxml())

        if 'id' in vars(self): p.removeBody(self.id)
        self.id = p.loadURDF(base_temp, self.pos, p.getQuaternionFromEuler(self.rot),useFixedBase=True)
        f.close()


        if ee_kind == 'Gripper': self.end_effector_obj = Gripper(self.id,gears)
        else: self.end_effector_obj = Suction(self.id)

        for j in range(7):
            ji = p.getJointInfo(self.id, j)
            jointName,jointType = ji[1],ji[2]
            if (jointType == p.JOINT_REVOLUTE):
                p.resetJointState(self.id, j, self.reset_joint_poses[j])
                i+=1
        
        return self.id

    def set_end_effector(self,base):        
        self.end_effector = base
        if 'id' not in vars(self): return
        self.set_base(self.base)

    def set_pick_pose(self):
        pass

    def plan(self,origin,target,rot,dt=1):
        ee_pos = origin
        o_pos,o_rot = target,rot

        route = list()
        route.append((o_pos,o_rot))

        points = list()
        for o_pos,o_rot in route:
            o_pos = np.array(o_pos)
            
            while np.linalg.norm(o_pos - ee_pos) != 0:
                direction = o_pos - ee_pos
                length = abs(np.linalg.norm(direction))
                if length < (self.scene.timestep * dt):  
                    increment_direction = direction
                else: 
                    increment_direction = direction / length * (self.scene.timestep * dt)
                ee_pos += increment_direction
                p.addUserDebugPoints([ee_pos],[[1,0,0]],2,lifeTime=5)

                points.append((np.array(ee_pos),np.array(o_rot)))
        return points

    def signal_pick_plan(self,camera,objs):
        c_pos,c_rot,fov,focal,d_piexls,pixels = camera

        def key(o):
            o_pos,o_rot = o
            e_pos,e_orn,_,_,_,_ = p.getLinkState(self.id,p.getNumJoints(self.id)-1)
            o_pos,e_pos = np.array(o_pos),np.array(e_pos)

            distance = abs(np.linalg.norm(o_pos - e_pos))
            return o_pos[2] * 10 - distance

        o = max(objs,key=key)
        o_pos,o_rot = o
        o_pos = np.array(o_pos)
        
        pick_pos,pick_rot = self.pick_pose['pos'],self.pick_pose['rot']
        pick_rot = Rotation.from_euler("xyz",o_rot).apply(Rotation.from_euler("xyz",pick_rot).as_rotvec())

        num_joints = p.getNumJoints(self.id)
        ee_index = num_joints-1
        ee_pos,ee_orn,_,_,_,_ = p.getLinkState(self.id,ee_index)
        ee_pos = np.array(ee_pos)

        points = self.plan(ee_pos,o_pos + [0,0,0.15],pick_rot)
        points.extend(self.plan(o_pos + [0,0,0.15],o_pos+pick_pos,pick_rot))
        def output(): self.result = None,points
        self.actions.append((output,()))

    def signal_move(self,*args,**kwargs):
        def task(ee_pos,ee_rot):
            pandaNumDofs = len(self.reset_joint_poses)
            ll = [-7]*pandaNumDofs
            #upper limits for null space (todo: set them to proper range)
            ul = [7]*pandaNumDofs
            #joint ranges for null space (todo: set them to proper range)
            jr = [7]*pandaNumDofs
            ee_orn = p.getQuaternionFromEuler(ee_rot)
            num_joints = p.getNumJoints(self.id)
            ee_index = num_joints-1
            poses = p.calculateInverseKinematics(self.id, ee_index, ee_pos, ee_orn,lowerLimits=ll,upperLimits=ul,jointRanges=jr,restPoses=self.reset_joint_poses,jointDamping=self.joint_damping,maxNumIterations=20)
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
            # p.addUserDebugLine(pos,axis_x,[1,0,0],2,lifeTime=5)
            # p.addUserDebugLine(pos,axis_y,[0,1,0],2,lifeTime=5)
            # p.addUserDebugLine(pos,axis_z,[0,0,1],2,lifeTime=5)

        num_joints = p.getNumJoints(self.id)
        ee_index = num_joints-1
        ee_pos,ee_orn,_,_,_,_ = p.getLinkState(self.id,ee_index)
        ee_pos = np.array(ee_pos)

        points = list()
        if 'absolute' == kwargs["mode"]:
            target = kwargs["pos"]
            ee_rot = p.getEulerFromQuaternion(ee_orn)
            points = self.plan(ee_pos,target,ee_rot)
        elif 'relative' == kwargs["mode"]:
            off_pos = kwargs["pos"]
            ee_rot = p.getEulerFromQuaternion(ee_orn)
            points = self.plan(ee_pos,ee_pos + off_pos,ee_rot)
        else:
            points = args[0]

        for ee_pos,ee_rot in points: self.actions.append((task,(ee_pos,ee_rot)))
        
        def output(): self.result = None,
        self.actions.append((output,()))
        pass
    
    def signal_do(self,**kwargs):
        self.end_effector_obj.do(kwargs['pickup'])
        def output(): self.result = None,
        self.actions.append((output,()))
        pass

    def signal_place(self,origin,pick_rot,target,rot):
        def output(): self.result = None,
        self.actions.append((output,()))
        pass