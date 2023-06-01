import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation
import xml.dom.minidom as xml
import os 

from digitaltwin.active_obj import ActiveObject
from digitaltwin.end_effector import Gripper,Suction

class Robot(ActiveObject):
    def __init__(self,scene,**kwargs):
        if 'reset_joint_poses' not in kwargs: kwargs['reset_joint_poses'] = []
        if 'joint_damping' not in kwargs: kwargs['joint_damping'] = []
        if 'end_effector' not in kwargs: kwargs['end_effector'] = ''
        if 'speed' not in kwargs:  kwargs['speed'] = 1.0
        self.speed = kwargs['speed']
        self.current_joint_poses = self.reset_joint_poses = kwargs['reset_joint_poses']
        self.joint_damping = kwargs['joint_damping']
        self.end_effector = kwargs['end_effector']
        
        super().__init__(scene, **kwargs)
        self.pickup = False
        pass
    
    def properties(self):
        properties = super().properties()
        
        x,y,z,rx,ry,rz = self.get_end_effector_pose()
        properties.update(kind='Robot',speed=self.speed,
                          end_effector=self.end_effector,
                          joint_damping=self.joint_damping,
                          current_joint_poses=self.current_joint_poses,
                          reset_joint_poses=self.reset_joint_poses,
                          end_effector_pos=[x,y,z],end_effector_rot=[rx,ry,rz])
        return properties

    def update(self,dt):
        super().update(dt)
        if self.end_effector_obj:
            self.end_effector_obj.update(dt)
            if not self.end_effector_obj.idle: self.actions.append((lambda *args:None,()))
        pass

    def restore(self):
        super().restore()
        p.setJointMotorControlArray(self.id, self.active_joint_indexes, p.POSITION_CONTROL, self.reset_joint_poses)
        self.end_effector_obj.do(False)

    def set_base(self,base):
        base_temp = self.base = base
        
        ee_kind = ""
        mimic_name = ''
        gears = []
        with xml.parse(os.path.join(self.scene.data_dir,base)) as doc_robot:
            node_robot = doc_robot.getElementsByTagName('robot')[0]
            for mesh in node_robot.getElementsByTagName('mesh'):
                filename = mesh.getAttribute('filename')
                mesh.setAttribute('filename',os.path.join(self.scene.data_dir,os.path.dirname(base),filename))

            num_joints = len(node_robot.getElementsByTagName('link'))

            if self.end_effector:
                link_ee = node_robot.getElementsByTagName('link')[-1]
                link_ee_name = link_ee.getAttribute('name')
                with xml.parse(os.path.join(self.scene.data_dir,self.end_effector)) as doc_ee:
                    node_ee = doc_ee.getElementsByTagName('robot')[0]
                    ee_kind = node_ee.getAttribute("kind")
                    for mesh in node_ee.getElementsByTagName('mesh'):
                        filename = mesh.getAttribute('filename')
                        mesh.setAttribute('filename',os.path.join(self.scene.data_dir,os.path.dirname(self.end_effector),filename))
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
                        multiplier = int(node_mimic.getAttribute('multiplier')) if node_mimic.hasAttribute('multiplier') else 1
                        offset = 0#int(node_mimic.getAttribute('offset'))
                        gears.append((mimic_name,joint_name,multiplier))
                node_robot.removeChild(link_ee).unlink()

            import tempfile
            f = tempfile.NamedTemporaryFile("w",delete=False)
            base_temp = f.name
            f.write(node_robot.toxml())
            f.close()

        if 'id' in vars(self): p.removeBody(self.id)
        print('robot urdf',base_temp,flush=True)
        self.id = p.loadURDF(base_temp, self.pos, p.getQuaternionFromEuler(self.rot),useFixedBase=True)
        if ee_kind == 'Gripper': self.end_effector_obj = Gripper(self.id,gears)
        elif ee_kind == 'Suction': self.end_effector_obj = Suction(self.id)
        else: 
            class EndEffector:
                def __init__(self): self.idle = True
                def update(self,dt):pass
                def do(self,pickup):pass
                def get_properties(self): return dict(king='EndEffector')
            self.end_effector_obj = EndEffector()
            pass

        self.used_joint_indexes = []
        self.active_joint_indexes = []
        self.joint_velocities = []
        for i in range(p.getNumJoints(self.id)):
            p.setCollisionFilterPair(self.scene.plane,self.id,-1,i,0)
            ji = p.getJointInfo(self.id, i)
            jointName,jointType,jointVelocity = ji[1],ji[2],ji[11]
            if (jointType != p.JOINT_REVOLUTE and jointType != p.JOINT_PRISMATIC and jointType != p.JOINT_SPHERICAL): continue
            self.active_joint_indexes.append(i)
            self.joint_velocities.append(jointVelocity)
            if i < num_joints: self.used_joint_indexes.append(i)
        
        if len(self.active_joint_indexes) > len(self.joint_damping): self.joint_damping.extend([0]*(len(self.active_joint_indexes)-len(self.joint_damping)))
        else: self.joint_damping = self.joint_damping[:len(self.active_joint_indexes)]

        if len(self.used_joint_indexes) > len(self.reset_joint_poses): self.reset_joint_poses.extend([0]*(len(self.used_joint_indexes)-len(self.reset_joint_poses)))
        else: self.reset_joint_poses = self.reset_joint_poses[:len(self.used_joint_indexes)]

        self.set_joints(self.reset_joint_poses)
        
        if 'end_effector_axis' in vars(self):
            p.removeUserDebugItem(self.end_effector_axis[0])
            p.removeUserDebugItem(self.end_effector_axis[1])
            p.removeUserDebugItem(self.end_effector_axis[2])

        pos,orn,_,_,_,_ = p.getLinkState(self.id,num_joints-1)
        axis_x = Rotation.from_quat(orn).apply(np.array([0.05,0,0])) + pos
        axis_y = Rotation.from_quat(orn).apply(np.array([0,0.05,0])) + pos
        axis_z = Rotation.from_quat(orn).apply(np.array([0,0,0.05])) + pos
        self.end_effector_axis = p.addUserDebugLine(pos,axis_x,[1,0,0],2),p.addUserDebugLine(pos,axis_y,[0,1,0],2),p.addUserDebugLine(pos,axis_z,[0,0,1],2)

        self.home_pos,self.home_rot = pos,p.getEulerFromQuaternion(orn)
    
    def set_speed(self,value):
        if round(value) == 0.000: return 
        self.speed = value
     
    def get_joints(self):
        return self.current_joint_poses[:len(self.used_joint_indexes)]
    
    def set_joints(self,joint_poses):
        joint_poses = joint_poses[:len(self.used_joint_indexes)]
        for i in self.used_joint_indexes: p.resetJointState(self.id, i, joint_poses[i])
        p.setJointMotorControlArray(self.id, self.used_joint_indexes, p.POSITION_CONTROL, joint_poses)
        self.current_joint_poses = joint_poses[:len(self.used_joint_indexes)]
    
    def set_end_effector_pose(self,pose):
        num_joints = p.getNumJoints(self.id)
        ee_index = num_joints-1
        joint_poses = p.calculateInverseKinematics(self.id, ee_index, pose[0:3], p.getQuaternionFromEuler(pose[3:6]),restPoses=self.reset_joint_poses,maxNumIterations=200)
        self.set_joints(list(joint_poses)[:len(self.used_joint_indexes)])

    def get_end_effector_pose(self):
        num_joints = p.getNumJoints(self.id)
        ee_pos,ee_orn,_,_,_,_ = p.getLinkState(self.id,num_joints-1)
        ee_rot = p.getEulerFromQuaternion(ee_orn)
        return [ee_pos[0],ee_pos[1],ee_pos[2],ee_rot[0],ee_rot[1],ee_rot[2]]

    def set_end_effector(self,base):
        if 'id' not in vars(self): return
        self.end_effector = base
        self.set_base(self.base)
        
    def plan(self,origin,target,s=1):
        ee_pos,ee_rot = origin
        pick_pos,pick_rot = target
        pick_orn = p.getQuaternionFromEuler(pick_rot)
        
        route_poses = list()
        end_poses = p.calculateInverseKinematics(self.id, p.getNumJoints(self.id)-1, pick_pos, pick_orn, restPoses=self.reset_joint_poses, maxNumIterations=200)
        end_poses = list(end_poses)[:len(self.used_joint_indexes)]
        
        num = int(1 / s / self.scene.timestep)
        for n in range(num):
            poses = []
            t = n / num
            for i in range(len(self.current_joint_poses)):
                poses.append(np.interp(t,[0,1],[self.current_joint_poses[i],end_poses[i]]))
            
            route_poses.append(poses[:len(self.used_joint_indexes)])
        return route_poses

    def signal_pick_plan(self,*args,**kwargs):
        objs, = args
        def near_obj(o):
            e_pos,e_orn,_,_,_,_ = p.getLinkState(self.id,p.getNumJoints(self.id)-1)
            o_pos,e_pos = np.array(o[0]),np.array(e_pos)
            distance = abs(np.linalg.norm(o_pos - e_pos))
            return o_pos[2] * 100 - distance
        
        o_pos,o_rot,o_mesh = max(objs,key=near_obj)
        o_pos = np.array(o_pos)
        o_r = Rotation.from_euler("xyz",o_rot)

        def perfect_pick_pose(pick_pose):
            pick_p = np.array(pick_pose['pos'])
            pick_r = Rotation.from_euler("xyz",pick_pose['rot'])
            o = o_pos + o_r.apply(pick_p)
            # pick_x = o_pos + o_r.apply(pick_p + [0.15,0,0])
            # pick_y = o_pos + o_r.apply(pick_p + [0,0.15,0])
            # pick_z = o_pos + o_r.apply(pick_p + [0,0,0.15])
            # p.addUserDebugLine(o,pick_x,[1,0,0],1,lifeTime=0)
            # p.addUserDebugLine(o,pick_y,[0,1,0],1,lifeTime=0)
            # p.addUserDebugLine(o,pick_z,[0,0,1],1,lifeTime=0)
            pick_x = o_pos + o_r.apply(pick_p + pick_r.apply([0.1,0,0]))
            pick_y = o_pos + o_r.apply(pick_p + pick_r.apply([0,0.1,0]))
            pick_z = o_pos + o_r.apply(pick_p + pick_r.apply([0,0,0.1]))
            p.addUserDebugLine(o,pick_z,[0,0,1],5,lifeTime=5)
            
            axis_up = [0,0,1]
            pick_axis_up = (o_r * pick_r).apply([0,0,1])
            same = np.dot(axis_up,pick_axis_up) / np.linalg.norm(axis_up) * np.linalg.norm(pick_axis_up)
            return same
        
        pose = max(kwargs['pick_poses'],key=perfect_pick_pose)
        pick_pos,pick_rot = pose['pos'],pose['rot']
        pick_pos = o_pos + o_r.apply(pick_pos)
        pick_r = o_r * Rotation.from_euler("xyz",pick_rot)
        pick_rot = pick_r.as_euler('xyz')

        num_joints = p.getNumJoints(self.id)
        ee_pos,ee_orn,_,_,_,_ = p.getLinkState(self.id,num_joints-1)
        ee_pos = np.array(ee_pos)
        ee_rot = p.getEulerFromQuaternion(ee_orn)

        route_poses = self.plan( (ee_pos,pick_rot), (pick_pos, pick_rot))

        def output(): self.result = None,route_poses,None
        self.actions.append((output,()))

    def signal_plan_move(self,*args,**kwargs):
        num_joints = p.getNumJoints(self.id)
        ee_index = num_joints-1

        def task(*poses):
            p.setJointMotorControlArray(self.id, self.used_joint_indexes, p.POSITION_CONTROL, poses)
            self.current_joint_poses = poses

        def task2(*point):
            num_joints = p.getNumJoints(self.id)
            ee_index = num_joints-1
            
            poses = p.calculateInverseKinematics(self.id, ee_index, point, p.getQuaternionFromEuler([0,0,0]),
                                                restPoses=self.current_joint_poses,jointDamping=self.joint_damping,maxNumIterations=200)[:len(self.used_joint_indexes)]
            p.setJointMotorControlArray(self.id, self.used_joint_indexes, p.POSITION_CONTROL, poses)

            self.current_joint_poses = list(poses)
            p.addUserDebugPoints([point],[[1,1,1]],2,lifeTime=5)

        route_poses,route_points = args
        if route_poses:
            for poses in route_poses: self.actions.append((task,poses))
        else:
            for point in route_points: self.actions.append((task2,point))
        
        def output(last_pos):
            pos,orn,_,_,_,_ = p.getLinkState(self.id,ee_index)
            pos = np.linalg.norm(pos)
            if round(last_pos - pos,6) != 0.000000:
                self.actions.append((output,(pos,)))
                return
            self.result = None,
        self.actions.append((output,(0,)))
        pass

    def signal_pick_move(self,pick_points,**kwargs):
        num_joints = p.getNumJoints(self.id)
        ee_index = num_joints-1

        ee_pos,ee_orn,_,_,_,_ = p.getLinkState(self.id,num_joints-1)
        ee_pos = np.array(ee_pos)
        ee_rot = p.getEulerFromQuaternion(ee_orn)

        if not pick_points: 
            self.result = 'failed', 
            return

        if 'speed' not in kwargs: kwargs['speed'] = 1
        speed = kwargs['speed']

        pick_pos,pick_rot = pick_points[0]
        pick_orn = p.getQuaternionFromEuler(pick_rot)
        route_poses = self.plan( (ee_pos,pick_rot), (pick_pos, pick_rot), speed)

        if 'axis' in vars(self):
            p.removeUserDebugItem(self.axis[0])
            p.removeUserDebugItem(self.axis[1])
            p.removeUserDebugItem(self.axis[2])
        
        axis_x = Rotation.from_euler('xyz',pick_rot).apply([0.05,0,0]) + pick_pos
        axis_y = Rotation.from_euler('xyz',pick_rot).apply([0,0.05,0]) + pick_pos
        axis_z = Rotation.from_euler('xyz',pick_rot).apply([0,0,0.05]) + pick_pos
        self.axis = [
            p.addUserDebugLine(pick_pos,axis_x,[1,0,0],2),
            p.addUserDebugLine(pick_pos,axis_y,[0,1,0],2),
            p.addUserDebugLine(pick_pos,axis_z,[0,0,1],2)
        ]

        def task(*poses):
            p.setJointMotorControlArray(self.id, self.used_joint_indexes, p.POSITION_CONTROL, poses)
            self.current_joint_poses = poses

        for poses in route_poses: self.actions.append((task,poses))
        def output(last_pos): self.result = None,
        self.actions.append((output,(0,)))
        pass

    def signal_move(self,*args,**kwargs):
        if 'home' in kwargs:
            self.signal_home()
            return 
        
        num_joints = p.getNumJoints(self.id)
        ee_index = num_joints-1
        ee_pos,ee_orn,_,_,_,_ = p.getLinkState(self.id,ee_index)
        ee_pos = np.array(ee_pos)
        ee_rot = p.getEulerFromQuaternion(ee_orn)
        t_pos = ee_pos
        t_rot = ee_rot
        poses = []
        
        def task(poses):
            p.setJointMotorControlArray(self.id, self.used_joint_indexes, p.POSITION_CONTROL, poses)
            self.current_joint_poses = poses

        if 'speed' not in kwargs: kwargs['speed'] = 1
        speed = kwargs['speed']

        if 'mode' in kwargs:
            if 'point' == kwargs['mode']:
                t_pos = kwargs['point'][0:3]
                t_rot = kwargs['point'][3:6]
                route_poses = self.plan((ee_pos,ee_rot),(t_pos,t_rot))
                for poses in route_poses: self.actions.append((task,(poses,)))
            else:
                poses = kwargs['joints'][:len(self.used_joint_indexes)]
                self.actions.append((task,(poses,)))
        else:
            route_poses = self.plan((ee_pos,ee_rot),(t_pos,t_rot),speed)
            for poses in route_poses: self.actions.append((task,(poses,)))
            
        def output(last_pos): self.result = None,
        self.actions.append((output,(0,)))
        pass
    
    def signal_move_relatively(self,*args,**kwargs):
        num_joints = p.getNumJoints(self.id)
        ee_index = num_joints-1
        ee_pos,ee_orn,_,_,_,_ = p.getLinkState(self.id,ee_index)
        ee_pos = np.array(ee_pos)
        ee_rot = np.array(p.getEulerFromQuaternion(ee_orn))
        t_pos = ee_pos
        t_rot = ee_rot

        if 'mode' in kwargs:
            if 'point' == kwargs['mode']:
                t_pos = kwargs['point'][0:3] + ee_pos
                t_rot = kwargs['point'][3:6] + ee_rot
            else:
                poses = kwargs['joints']
        
        if 'speed' not in kwargs: kwargs['speed'] = 1
        speed = kwargs['speed']

        def task(poses):
            p.setJointMotorControlArray(self.id, self.used_joint_indexes, p.POSITION_CONTROL, poses)
            self.current_joint_poses = poses

        route_poses = self.plan((ee_pos,ee_rot),(t_pos,t_rot))
        for poses in route_poses: self.actions.append((task,(poses,)))
        
        def output(last_pos): self.result = None,
        self.actions.append((output,(0,)))
        pass
    
    def signal_do(self,*args,**kwargs):
        self.end_effector_obj.do(kwargs['pickup'])
        pass

    def signal_pick(self,*args,**kwargs):
        self.end_effector_obj.do(True)
        pass
    
    def signal_place(self,*args,**kwargs):
        self.end_effector_obj.do(False)
        pass
    
    def signal_home(self):
        num_joints = p.getNumJoints(self.id)
        ee_index = num_joints-1
        ee_pos,ee_orn,_,_,_,_ = p.getLinkState(self.id,ee_index)
        ee_pos = np.array(ee_pos)
        ee_rot = p.getEulerFromQuaternion(ee_orn)
        
        t_pos = self.home_pos
        t_rot = self.home_rot

        self.reset_joint_poses
        joint_poses = []
        for joint in self.used_joint_indexes:
            position,velocity,reaction_force,motor_torque = p.getJointState(self.id,joint)
            joint_poses.append(position)

        def task(poses):
            p.setJointMotorControlArray(self.id, self.used_joint_indexes, p.POSITION_CONTROL, poses)
            self.current_joint_poses = poses

        num = int(1 / self.speed / self.scene.timestep)
        for n in range(num):
            poses = []
            t = n / num
            for i in range(len(self.used_joint_indexes)):
                poses.append(np.interp(t,[0,1],[self.current_joint_poses[i],self.reset_joint_poses[i]]))
            self.actions.append((task,(poses,)))

        def output(): self.result = None,
        self.actions.append((output,()))
        pass