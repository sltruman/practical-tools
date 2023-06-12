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
        p.setJointMotorControlArray(self.id, self.used_joint_indexes, p.POSITION_CONTROL, self.reset_joint_poses)
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
        
        if 'lines' in vars(self):
            for line in self.lines:
                p.removeUserDebugItem(line)

        self.lines = list()

        pos,orn,_,_,_,_ = p.getLinkState(self.id,p.getNumJoints(self.id)-1)
        axis_x = Rotation.from_quat(orn).apply(np.array([0.05,0,0])) + pos
        axis_y = Rotation.from_quat(orn).apply(np.array([0,0.05,0])) + pos
        axis_z = Rotation.from_quat(orn).apply(np.array([0,0,0.05])) + pos
        self.lines.extend([p.addUserDebugLine(pos,axis_x,[1,0,0],2),
                          p.addUserDebugLine(pos,axis_y,[0,1,0],2),
                          p.addUserDebugLine(pos,axis_z,[0,0,1],2)])

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

    def end_pose_from_joint_poses(self,joint_poses):
        pos = np.array([0,0,0])
        rot = np.array([0,0,0])
        i = p.getNumJoints(self.id)-1
        
        while i > -1:
            _,_,l_pos,l_orn,_,_,_,_ = p.getLinkState(self.id,i,True)
            pos = pos + l_pos
            rot = (Rotation.from_quat(l_orn) * Rotation.from_euler('xyz',rot)).as_euler('xyz')
            _,_,_,_,_,_,_,_,_,_,_,_,_,j_axis,j_pos,j_orn,parent_link_index = p.getJointInfo(self.id, i)
            R = Rotation.from_quat(j_orn)
            if i in self.used_joint_indexes:
                R = R * Rotation.from_rotvec(np.array(j_axis) * joint_poses[self.used_joint_indexes.index(i)])

            pos = R.apply(pos) + j_pos
            rot = (R * Rotation.from_euler('xyz',rot)).as_euler('xyz')
            i = parent_link_index
            
        pos = Rotation.from_euler('xyz',self.rot).apply(pos) + self.pos
        rot = (Rotation.from_euler('xyz',self.rot) * Rotation.from_euler('xyz',rot)).as_euler('xyz')

        return [pos[0],pos[1],pos[2],rot[0],rot[1],rot[2]]
        
    def plan(self,target_pos,target_rot,s=1,mode='joint'):
        target_orn = p.getQuaternionFromEuler(target_rot)
        num_joints = p.getNumJoints(self.id)
        ee_index = num_joints-1

        ee_pos,ee_orn,_,_,_,_ = p.getLinkState(self.id,num_joints-1)
        ee_pos = np.array(ee_pos)
        ee_rot = p.getEulerFromQuaternion(ee_orn)
        
        route_poses = list()

        if mode=='joint':
            end_poses = p.calculateInverseKinematics(self.id, ee_index, target_pos, target_orn, restPoses=self.reset_joint_poses, maxNumIterations=200)
            end_poses = list(end_poses)[:len(self.used_joint_indexes)]
            num = round(1 / self.scene.timestep)
            for n in range(num):
                poses = []
                t = n / num
                for i in range(len(self.current_joint_poses)):
                    poses.append(np.interp(t,[0,1],[self.current_joint_poses[i],end_poses[i]]))
                route_poses.append(poses[:len(self.used_joint_indexes)])
        else:
            num = round(1 / self.scene.timestep)
            for n in range(num):
                poses = []
                t = n / num
                x = np.interp(t,[0,1],[ee_pos[0],target_pos[0]])
                y = np.interp(t,[0,1],[ee_pos[1],target_pos[1]])
                z = np.interp(t,[0,1],[ee_pos[2],target_pos[2]])
                pos = [x,y,z]

                q1 = np.array(ee_orn)
                q2 = np.array(target_orn)
                q_dot = q1.dot(q2)
    
                if (np.abs(q_dot) < 1e-5):
                    orn = q1
                else:
                    if q_dot < 0:
                        q2 = -q2
                    
                    orn = (1 - t) * q1 + t * q2
                    orn = orn / np.linalg.norm(orn)

                poses = p.calculateInverseKinematics(self.id, ee_index, pos, orn, restPoses=self.reset_joint_poses, maxNumIterations=200)
                route_poses.append(list(poses)[:len(self.used_joint_indexes)])
        
        # points = []
        # for poses in route_poses:
        #     point = self.end_pose_from_joint_poses(poses)
        #     points.append(point)

        # def task(i):
        #     self.lines.append(p.addUserDebugLine(points[i][0:3],points[i+1][0:3],[0,1,0],1,lifeTime=0))
            
        # for i in range(len(points)-1):
        #     self.actions.append((task,(i,)))
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

    def signal_pick_move(self,*args,**kwargs):
        if not args: 
            active_plugins = kwargs['plugins']
            picklight = active_plugins['PickLight']
            _,pickpose = picklight.result
            args = pickpose,
        
        num_joints = p.getNumJoints(self.id)
        ee_index = num_joints-1
        ee_pos,ee_orn,_,_,_,_ = p.getLinkState(self.id,ee_index)
        ee_pos = np.array(ee_pos)
        ee_rot = p.getEulerFromQuaternion(ee_orn)

        if 'speed' not in kwargs: kwargs['speed'] = 1
        speed = kwargs['speed']
        if speed == 0: 
            if 'pickup' in kwargs: self.end_effector_obj.do(kwargs['pickup'])
            self.result = None,[ee_pos[0],ee_pos[1],ee_pos[2],ee_rot[0],ee_rot[1],ee_rot[2]]
            return

        if 'mode' not in kwargs: kwargs['mode'] = 'joint'
        mode = kwargs['mode']

        pick_pose = args[0]
        pick_pos,pick_rot = pick_pose[0:3],pick_pose[3:6]
        pick_orn = p.getQuaternionFromEuler(pick_rot)
        route_poses = self.plan( pick_pos, pick_rot, speed, mode)
        
        axis_x = Rotation.from_euler('xyz',pick_rot).apply([0.05,0,0]) + pick_pos
        axis_y = Rotation.from_euler('xyz',pick_rot).apply([0,0.05,0]) + pick_pos
        axis_z = Rotation.from_euler('xyz',pick_rot).apply([0,0,0.05]) + pick_pos
        
        self.lines.extend([p.addUserDebugLine(pick_pos,axis_x,[1,0,0],2),
                           p.addUserDebugLine(pick_pos,axis_y,[0,1,0],2),
                           p.addUserDebugLine(pick_pos,axis_z,[0,0,1],2)])

        def task(*poses):
            p.setJointMotorControlArray(self.id, self.used_joint_indexes, p.POSITION_CONTROL, poses)
            self.current_joint_poses = poses

        for poses in route_poses: self.actions.append((task,poses))
        def output(last_pos): 
            pos,_,_,_,_,_,_,_ = p.getLinkState(self.id,ee_index,True)
            pos = np.linalg.norm(pos)
            if round(last_pos - pos,6) != 0.000000:
                self.actions.append((output,(pos,)))
                return
            
            if 'pickup' in kwargs: self.end_effector_obj.do(kwargs['pickup'])
            self.result = None,
        
        self.actions.append((output,(0,)))
        pass

    def signal_move(self,*args,**kwargs):
        if 'home' in kwargs and kwargs['home']:
            self.signal_home()
            return 
        
        num_joints = p.getNumJoints(self.id)
        ee_index = num_joints-1
        ee_pos,ee_orn,_,_,_,_ = p.getLinkState(self.id,ee_index)
        ee_pos = np.array(ee_pos)
        ee_rot = p.getEulerFromQuaternion(ee_orn)

        if 'speed' not in kwargs: kwargs['speed'] = 1
        speed = kwargs['speed']
        if speed == 0: 
            if 'pickup' in kwargs: self.end_effector_obj.do(kwargs['pickup'])
            self.result = None,[ee_pos[0],ee_pos[1],ee_pos[2],ee_rot[0],ee_rot[1],ee_rot[2]]
            return

        t_pos = ee_pos
        t_rot = ee_rot
        poses = []
        
        def task(poses):
            p.setJointMotorControlArray(self.id, self.used_joint_indexes, p.POSITION_CONTROL, poses)
            self.current_joint_poses = poses
        
        if 'mode' not in kwargs: kwargs['mode'] = 'joint'
        mode = kwargs['mode']

        if 'point' in kwargs:
            t_pos = kwargs['point'][0:3]
            t_rot = kwargs['point'][3:6]
            route_poses = self.plan(t_pos,t_rot,speed,mode)
            for poses in route_poses: self.actions.append((task,(poses,)))
        else:
            end_poses = kwargs['joints'][:len(self.current_joint_poses)]
            num = int(1 / speed / self.scene.timestep)
            route_poses = []
            for n in range(num):
                poses = []
                t = n / num
                for i in range(len(self.current_joint_poses)):
                    poses.append(np.interp(t,[0,1],[self.current_joint_poses[i],end_poses[i]]))
                self.actions.append((task,(poses,)))
            
        def output(last_pos): 
            pos,_,_,_,_,_ = p.getLinkState(self.id,ee_index)
            pos = np.linalg.norm(pos)
            if round(last_pos - pos,6) != 0.000000:
                self.actions.append((output,(pos,)))
                return
            
            if 'pickup' in kwargs: self.end_effector_obj.do(kwargs['pickup'])
            self.result = None,
        
        self.actions.append((output,(0,)))
        pass    
    
    def signal_move_relatively(self,*args,**kwargs):
        num_joints = p.getNumJoints(self.id)
        ee_index = num_joints-1
        ee_pos,ee_orn,_,_,_,_ = p.getLinkState(self.id,ee_index)
        ee_rot = p.getEulerFromQuaternion(ee_orn)


        if args: 
            pick_point = args[0]
            ee_pos = pick_point[0:3]
            ee_rot = pick_point[3:6]
            ee_orn = p.getQuaternionFromEuler(ee_rot)

        if 'target' not in kwargs: kwargs['target'] = 'task_current'
        target = kwargs['target']

        if target == 'task_next':
            declare = kwargs['declare']
            cursor = kwargs['cursor']
            active_plugins = kwargs['plugins']

            act = declare[cursor]
            next = act['next']
            act_next = declare[next]
            
            if act_next['fun'] == 'move_relatively':
                self.result = 'link error.', 
                return
            elif act_next['fun'] == 'move':
                move_args = act_next['args']

                if 'point' in move_args:
                    pick_pose = move_args['point']
                elif 'joints' in move_args:
                    joints = move_args['joints']
                    n = len(self.used_joint_indexes) - len(joints)
                    if n > 0: joints.extend([0] * n)
                    pick_pose = self.end_pose_from_joint_poses(joints)
                else:
                    self.result = 'parameters error.', 
                    return
            elif act_next['fun'] == 'pick_move':
                picklight = active_plugins['PickLight']
                _,pick_pose = picklight.result

                if not pick_pose: 
                    self.result = 'failed', 
                    return
            
            ee_pos = pick_pose[0:3]
            ee_rot = pick_pose[3:6]
            ee_orn = p.getQuaternionFromEuler(ee_rot)

        if 'speed' not in kwargs: kwargs['speed'] = 1
        speed = kwargs['speed']
        if speed == 0: 
            if 'pickup' in kwargs: self.end_effector_obj.do(kwargs['pickup'])
            self.result = None,[ee_pos[0],ee_pos[1],ee_pos[2],ee_rot[0],ee_rot[1],ee_rot[2]]
            return

        if 'mode' not in kwargs: kwargs['mode'] = 'joint'
        mode = kwargs['mode']

        t_pos = ee_pos
        t_rot = ee_rot

        if 'point' in kwargs:
            t_pos = kwargs['point'][0:3] + np.array(ee_pos)
            t_rot = kwargs['point'][3:6] + np.array(ee_rot)
        
        route_poses = self.plan(t_pos,t_rot,speed,mode)

        def task(poses):
            p.setJointMotorControlArray(self.id, self.used_joint_indexes, p.POSITION_CONTROL, poses)
            self.current_joint_poses = poses

        for poses in route_poses: self.actions.append((task,(poses,)))
        
        def output(last_pos): 
            pos,_,_,_,_,_ = p.getLinkState(self.id,ee_index)
            pos = np.linalg.norm(pos)
            if round(last_pos - pos,6) != 0.000000:
                self.actions.append((output,(pos,)))
                return
            
            if 'pickup' in kwargs: self.end_effector_obj.do(kwargs['pickup'])
            self.result = None,
        
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
        if 'lines' in vars(self):
            for line in self.lines:
                p.removeUserDebugItem(line)
        
        self.lines = list()

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

        num = int(1 / self.speed / self.scene.timestep) + 1
        for n in range(num):
            poses = []
            t = n / num
            for i in range(len(self.used_joint_indexes)):
                poses.append(np.interp(t,[0,1],[self.current_joint_poses[i],self.reset_joint_poses[i]]))
            self.actions.append((task,(poses,)))

        def output(): self.result = None,
        self.actions.append((output,()))
        pass