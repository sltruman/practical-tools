import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation
import xml.dom.minidom as xml
import os
import pymeshlab as meshlab
import open3d

from .active_obj import ActiveObject

class Printer3D(ActiveObject):
    def __init__(self,scene,**kwargs):
        self.current_joint_poses = self.reset_joint_poses = kwargs['reset_joint_poses']
        self.joint_damping = kwargs['joint_damping']
        self.end_effector = kwargs['end_effector']
        kwargs['pos'][1] -= 0.5
        super().__init__(scene, **kwargs)
        pass

    def properties(self):
        info = super().properties()
        info.update(dict(kind='Printer3D',end_effector=self.end_effector))
        return info
    
    def update(self,dt):
        super().update(dt)
        if self.end_effector_obj: self.end_effector_obj.update(dt)
        pass

    def set_base(self,base):
        self.base = base
        
        import tempfile
        f = tempfile.NamedTemporaryFile("w")
        base_temp = f.name

        ee_kind = ''
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
                        offset = int(node_mimic.getAttribute('offset'))
                        gears.append((mimic_name,joint_name,multiplier))

                node_robot.removeChild(link_ee).unlink()
            f.write(node_robot.toxml())

        if 'id' in vars(self): p.removeBody(self.id)
        self.id = p.loadURDF(base_temp, self.pos, p.getQuaternionFromEuler(self.rot),useFixedBase=True)
        f.close()

        class PlaceHolder:
            def update(self,dt):pass
            def do(self,pickup):pass
        self.end_effector_obj = PlaceHolder()

        num_joints = p.getNumJoints(self.id)
        self.active_joints = []
        for i in range(num_joints):
            ji = p.getJointInfo(self.id, i)
            jointName,jointType = ji[1],ji[2]
            if (jointType != p.JOINT_REVOLUTE and jointType != p.JOINT_PRISMATIC and jointType != p.JOINT_SPHERICAL): continue
            if i < len(self.reset_joint_poses): p.resetJointState(self.id, i, self.reset_joint_poses[i])
            self.active_joints.append(i)

        if len(self.active_joints) > len(self.joint_damping): 
            self.joint_damping.extend([0]*(len(self.active_joints)-len(self.joint_damping)))
            self.reset_joint_poses.extend([0]*(len(self.active_joints)-len(self.reset_joint_poses)))
        else: 
            self.joint_damping = self.joint_damping[:len(self.active_joints)]
            self.reset_joint_poses = self.reset_joint_poses[:len(self.active_joints)]
        
        pos,orn,_,_,_,_ = p.getLinkState(self.id,num_joints-1)
        axis_x = Rotation.from_quat(orn).apply(np.array([0.05,0,0])) + pos
        axis_y = Rotation.from_quat(orn).apply(np.array([0,0.05,0])) + pos
        axis_z = Rotation.from_quat(orn).apply(np.array([0,0,0.05])) + pos
        p.addUserDebugLine(pos,axis_x,[1,0,0],2,lifeTime=0)
        p.addUserDebugLine(pos,axis_y,[0,1,0],2,lifeTime=0)
        p.addUserDebugLine(pos,axis_z,[0,0,1],2,lifeTime=0)
        self.home_pos,self.home_rot = pos,p.getEulerFromQuaternion(orn)
        return self.id

    def set_end_effector(self,base):        
        self.end_effector = base
        if 'id' not in vars(self): return
        self.set_base(self.base)

    def plan(self,origin,target,dt=1):
        ee_pos,ee_rot = origin
        o_pos,o_rot = target
        o_pos = np.array(o_pos)
        p.addUserDebugLine(ee_pos,o_pos,[1,1,1],2,lifeTime=5)

        route_poses = list()
        while np.linalg.norm(o_pos - ee_pos) != 0:
            direction = o_pos - ee_pos
            length = abs(np.linalg.norm(direction))
            if length < (self.scene.timestep * dt): 
                increment_direction = direction
            else: 
                increment_direction = direction / length * (self.scene.timestep * dt)
            ee_pos += increment_direction

            num_joints = p.getNumJoints(self.id)
            ee_index = num_joints-1
            ee_orn = p.getQuaternionFromEuler(o_rot)
            ll = [-7]*len(self.active_joints)
            ul = [7]*len(self.active_joints)
            jr = [7]*len(self.active_joints)
            
            poses = p.calculateInverseKinematics(self.id, ee_index, ee_pos, ee_orn,
                                                lowerLimits=ll,upperLimits=ul,jointRanges=jr,restPoses=self.reset_joint_poses,
                                                jointDamping=self.joint_damping,maxNumIterations=200)
            route_poses.append(poses)
        return route_poses

    def signal_move(self,*args,**kwargs):
        num_joints = p.getNumJoints(self.id)
        ee_index = num_joints-1
        ee_pos,ee_orn,_,_,_,_ = p.getLinkState(self.id,ee_index)
        ee_pos = np.array(ee_pos)
        ee_rot = p.getEulerFromQuaternion(ee_orn)

        t_pos = kwargs["x"],kwargs["y"],kwargs["z"]
        if 'rx' in kwargs and 'ry' in kwargs and 'rz' in kwargs:
            t_rot = kwargs["rx"],kwargs["ry"],kwargs["rz"]
        else:
            t_rot = ee_rot
        
        def task(*poses):
            p.setJointMotorControlArray(self.id, self.active_joints, p.POSITION_CONTROL, poses)
            self.current_joint_poses = poses
            pos,orn,_,_,_,_ = p.getLinkState(self.id,ee_index)
            axis_x = Rotation.from_quat(orn).apply([0.05,0,0]) + pos
            axis_y = Rotation.from_quat(orn).apply([0,0.05,0]) + pos
            axis_z = Rotation.from_quat(orn).apply([0,0,0.05]) + pos
            # p.addUserDebugLine(pos,axis_x,[1,0,0],2,lifeTime=0.1)
            # p.addUserDebugLine(pos,axis_y,[0,1,0],2,lifeTime=0.1)
            # p.addUserDebugLine(pos,axis_z,[0,0,1],2,lifeTime=0.1)
    
        route_poses = self.plan((ee_pos,ee_rot),(t_pos,t_rot))
        for poses in route_poses: self.actions.append((task,(poses)))
        
        def output(*poses):
            joint_poses = []
            for joint in self.active_joints:
                position,velocity,reaction_force,motor_torque = p.getJointState(self.id,joint)
                joint_poses.append(position)
            
            for i in range(len(poses)):
                if abs(joint_poses[i] - poses[i]) > 0.01:
                    self.actions.append((output,poses))
                    return

            self.result = None,
        self.actions.append((output,route_poses[-1]))
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
        for joint in self.active_joints:
            position,velocity,reaction_force,motor_torque = p.getJointState(self.id,joint)
            joint_poses.append(position)

        def task(*poses):
            p.setJointMotorControlArray(self.id, self.active_joints, p.POSITION_CONTROL, poses)
            pos,orn,_,_,_,_ = p.getLinkState(self.id,ee_index)
            axis_x = Rotation.from_quat(orn).apply([0.05,0,0]) + pos
            axis_y = Rotation.from_quat(orn).apply([0,0.05,0]) + pos
            axis_z = Rotation.from_quat(orn).apply([0,0,0.05]) + pos

        num = int(1 / self.scene.timestep) 
        for t in range(num):
            poses = []
            t /= num
            for i in range(len(self.reset_joint_poses)): 
                pose = (1 - t) * joint_poses[i] + t * self.reset_joint_poses[i]
                poses.append(pose)
            self.actions.append((task,(poses)))
        
        def output(): self.result = None,
        self.actions.append((output,()))
        pass

    def signal_print(self,*args,**kwargs):
        ms = meshlab.MeshSet()
        ms.load_new_mesh(self.product)
        ms.generate_sampling_poisson_disk(radius=meshlab.Percentage(1))
        ms.save_current_mesh ('convex_hull.ply')

        point_cloud = open3d.io.read_point_cloud('convex_hull.ply')
        open3d.visualization.draw_geometries([point_cloud])

        def output(): self.result = None,
        self.actions.append((output,()))
        pass