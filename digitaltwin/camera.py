import pybullet as p
from scipy.spatial.transform import Rotation
from .active_obj import ActiveObject

import pymeshlab as meshlab
import numpy as np
import sys
import socket as s
import os

class Camera3D(ActiveObject):
    def __init__(self,scene,**kwargs):
        super().__init__(scene, **kwargs)
        self.image_size = kwargs['image_size']
        self.forcal = kwargs['forcal']
        self.fov = kwargs['fov']
        self.sample_rate = kwargs['sample_rate']
        self.point_ids = list()
        pass
    
    def properties(self):
        info = super().properties()
        info.update(dict(kind='Camera3D',image_size=self.image_size,fov=self.fov,forcal=self.forcal))
        return info
    
    def rtt(self):
        import numpy as np
        num_joints = p.getNumJoints(self.id)
        pos,orn,_,_,_,_ = p.getLinkState(self.id,num_joints-1)
        pitch,roll,yaw = p.getEulerFromQuaternion(orn)

        near = self.forcal
        far = 1000
        origin = np.array(pos)
        viewport_length = np.tan(np.pi / 180 * self.fov / 2) * near * 2

        horizontal = np.array([viewport_length, 0, 0])
        vertical = np.array([0, 0, viewport_length])
        lower_left_corner = [0, near, 0] - horizontal/2 - vertical/2

        depth_far = near
        sample_rate = 2
        for j in range(sample_rate):
            for i in range(sample_rate):
                u = float(i) / (sample_rate-1)
                v = float(j) / (sample_rate-1)
                target = lower_left_corner + u*horizontal + v*vertical
                target = origin + Rotation.from_quat(orn).apply(target)
                
                l = np.linalg.norm(target - origin)
                dr = (target - origin) / l * far

                p.addUserDebugLine(origin, target + dr,[1,0,0],1,lifeTime=1)
                rayInfo = p.rayTest(origin, target + dr)
                if not rayInfo: continue
                
                id,linkindex,fraction,pos,norm = rayInfo[0]
                distance = np.linalg.norm(pos - origin)
                if depth_far < distance: depth_far = distance

        vm = p.computeViewMatrixFromYawPitchRoll(origin,near,180 / np.pi * yaw,180 / np.pi * pitch,180 / np.pi * roll,2)
        pm = p.computeProjectionMatrixFOV(self.fov,self.image_size[0]/self.image_size[1],near,far)
        _,_,pixels,depth_pixels,_ = p.getCameraImage(self.image_size[0],self.image_size[1],viewMatrix = vm,projectionMatrix = pm,renderer=p.ER_BULLET_HARDWARE_OPENGL)
        
        for h in range(0, self.image_size[1]):
            for w in range(0, self.image_size[0]):
                depth_pixels[h, w] = far * near / (far - (far - near) * depth_pixels[h, w])
        return pixels.tobytes(),depth_pixels.tobytes()

    def signal_capture(self,*args):
        def output(): self.result = (None,) + self.rtt()
        self.actions.append((output, ()))
        pass

    def signal_pose_recognize(self,*args):
        p.removeAllUserDebugItems()
        import numpy as np
        num_joints = p.getNumJoints(self.id)
        if num_joints: 
            pos,orn,_,_,_,_ = p.getLinkState(self.id,num_joints-1)
        else:
            pos,orn = p.getBasePositionAndOrientation(self.id)

        far = 1000
        origin = np.array(pos)
        viewport_length = np.tan(np.pi / 180 * self.fov / 2) * self.forcal * 2

        horizontal = np.array([viewport_length, 0, 0])
        vertical = np.array([0, viewport_length, 0])
        lower_left_corner = [0, 0, -self.forcal] - horizontal/2 - vertical/2
        ids = set()

        sample_rate = 2
        for j in range(sample_rate):
            for i in range(sample_rate):
                u = float(i) / (sample_rate-1)
                v = float(j) / (sample_rate-1)
                target = lower_left_corner + u*horizontal + v*vertical
                target = origin + Rotation.from_quat(orn).apply(target)
                l = np.linalg.norm(target - origin)
                dr = (target - origin) / l * far
                p.addUserDebugLine(origin, target + dr,[1,0,0],1,lifeTime=1)

        for j in range(self.sample_rate-1,0,-1):
            for i in range(self.sample_rate):
                u = float(i) / (self.sample_rate-1)
                v = float(j) / (self.sample_rate-1)
                target = lower_left_corner + u*horizontal + v*vertical
                target = origin + Rotation.from_quat(orn).apply(target)
                
                l = np.linalg.norm(target - origin)
                dr = (target - origin) / l * far
                
                def ray(origin,target):
                    rayInfo = p.rayTest(origin, target)
                    if not rayInfo: return
                    id,linkindex,fraction,pos,norm = rayInfo[0]
                    if id in self.scene.active_objs or id == 0: return
                    ids.add(id)
                self.actions.append((ray, (origin,target + dr)))
         
        def output():
            val = list()

            for id in ids:
                min,max = p.getAABB(id)
                min,max = np.array(min),np.array(max)
                center = (max - min) / 2 
                length = np.max(center)

                pos,orn = p.getBasePositionAndOrientation(id)
                rot = p.getEulerFromQuaternion(orn)
                mesh = None#p.getMeshData(id,flags=p.MESH_DATA_SIMULATION_MESH)
                
                axis_x = Rotation.from_quat(orn).apply(np.array([length,0,0])) + pos
                axis_y = Rotation.from_quat(orn).apply(np.array([0,length,0])) + pos
                axis_z = Rotation.from_quat(orn).apply(np.array([0,0,length])) + pos
                p.addUserDebugLine(pos,axis_x,[1,0,0],2,lifeTime=0)
                p.addUserDebugLine(pos,axis_y,[0,1,0],2,lifeTime=0)
                p.addUserDebugLine(pos,axis_z,[0,0,1],2,lifeTime=0)
                val.append((pos,rot,None))
            self.result = (None,val) if val else ('failed',)
        self.actions.append((output, ()))

class Camera3DReal(ActiveObject):
    def __init__(self,scene,**kwargs):
        self.point_ids = list()
        super().__init__(scene, **kwargs)
        self.image_size = kwargs['image_size']
        self.projection_transform = kwargs['projection_transform']
        self.eye_to_hand_transform = kwargs['eye_to_hand_transform']
        self.rgb_pixels = bytes()
        self.depth_pixels = bytes()
        self.sample_num = kwargs['sample_num']
        pass
    
    def properties(self):
        info = super().properties()
        info.update(dict(kind='Camera3DReal',image_size=self.image_size,projection_transform=self.projection_transform,eye_to_hand_transform=self.eye_to_hand_transform))
        return info
    
    def rtt(self):
        return self.rgb_pixels,self.depth_pixels

    def signal_capture(self,*args):
        sk = s.socket(s.AF_UNIX,s.SOCK_STREAM)

        try:
            tmp_dir = sys.argv[4]
            sock_path = os.path.join(tmp_dir,self.name + '.sock')
            sk.connect(sock_path)
        except:
            self.result = 'failed',
            return 
        
        def task():
            width = int.from_bytes(sk.recv(4),'little')
            height = int.from_bytes(sk.recv(4),'little')
            if not width:
                sk.close()
                self.result = 'failed',
                return

            rgb_pixels = bytes()
            depth_pixels = bytes()
            print('roi',width,height)
            size = width * height * 3
            while len(rgb_pixels) < size: 
                rgb_pixels += sk.recv(size - len(rgb_pixels))
            size = width * height * 4
            while len(depth_pixels) < size:
                depth_pixels += sk.recv(size - len(depth_pixels))

            self.rgb_pixels = rgb_pixels
            self.depth_pixels = depth_pixels

            sk.close()
            self.clear_point_clounds()
            self.actions.append((self.draw_point_cloud_from_depth_pixels,(rgb_pixels,depth_pixels,width,height)))
            self.result = None,self.eye_to_hand_transform

        self.actions.append((task,()))

    def set_calibration(self,projection_transform,eye_to_hand_transform):
        self.projection_transform = projection_transform
        self.eye_to_hand_transform = eye_to_hand_transform

        eye_to_hand_transform = np.array(eye_to_hand_transform)
        R = eye_to_hand_transform[:3, :3]
        T = eye_to_hand_transform[:3, 3]
        self.set_pos(T.tolist())
        self.set_rot(Rotation.from_matrix(R).as_euler('xyz').tolist())
        pass

    def draw_point_cloud_from_depth_pixels(self,rgb_pixels,depth_pixels,width,height):
        def depth_to_point_cloud(depth_image, projection_transform):
            projection_transform = np.array(projection_transform)
            rows, cols = depth_image.shape
            c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
            valid = (0 < depth_image) & (depth_image < 1000)
            z = np.where(valid, depth_image, np.nan)
            x = np.where(valid, z * (c - projection_transform[0, 2]) / projection_transform[0, 0], 0)
            y = np.where(valid, z * (r - projection_transform[1, 2]) / projection_transform[1, 1], 0)

            point_cloud = np.dstack((x, y, z))
            point_cloud = np.reshape(point_cloud, (rows * cols, 3))
            return point_cloud

        R = Rotation.from_euler('xyz',self.rot)
        T = self.pos
        count = width*height
        sample = int(count / self.sample_num)
        sample = 1 if sample < 1 else sample
        
        point_cloud_color = np.frombuffer(rgb_pixels, dtype=np.ubyte).reshape((width*height,3))[::sample] / 255
        depth_pixels = np.frombuffer(depth_pixels, dtype=np.float32).reshape((height,width))
        point_cloud = depth_to_point_cloud(depth_pixels,self.projection_transform)[::sample]
        point_cloud = R.apply(point_cloud) + T
        
        beg = 0; end = len(point_cloud)
        while beg < end:
            offset = end - beg
            if offset > 10000: offset = 10000
            point_id = p.addUserDebugPoints(point_cloud[beg:beg+offset],point_cloud_color[beg:beg+offset],1,lifeTime=0)
            self.point_ids.append(point_id)
            beg += offset
        pass

    def draw_point_cloud(self,ply_path):
        ms = meshlab.MeshSet()
        ms.load_new_mesh(ply_path)
        m = ms.current_mesh()
        vs = m.vertex_matrix()
        fs = m.face_matrix()
        vcs = m.vertex_color_matrix()[:,:3]
        R = Rotation.from_euler('xyz',self.rot)
        T = self.pos

        if len(vs) > len(vcs): vcs.extend([0,0,0]*(len(vs)-len(vcs)))
        point_cloud = R.apply(vs) + T
        
        beg = 0; end = len(vs)
        while beg < end:
            offset = end - beg
            if offset > 10000: offset = 10000
            point_id = p.addUserDebugPoints(point_cloud[beg:beg+offset],vcs[beg:beg+offset],1,lifeTime=0)
            self.point_ids.append(point_id)
            beg += offset
        pass

    def clear_point_clounds(self):
        for point_id in self.point_ids: p.removeUserDebugItem(point_id)
        pass