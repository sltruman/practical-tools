import pybullet as p
from scipy.spatial.transform import Rotation
from .active_obj import ActiveObject

import math
import numpy as np
import socket as s
import os

class Camera3D(ActiveObject):
    def __init__(self,scene,**kwargs):
        if 'focal' not in kwargs: kwargs['focal'] = 0.010
        if 'fov' not in kwargs: kwargs['fov'] = math.radians(20.4)
        if 'pixels_w' not in kwargs: kwargs['pixels_w'] = 1024
        if 'pixels_h' not in kwargs: kwargs['pixels_h'] = 768
        if 'sample_rate' not in kwargs: kwargs['sample_rate'] = 20
        if 'roi' not in kwargs: kwargs['roi'] = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

        super().__init__(scene, **kwargs)
        self.focal = self.profile['focal']
        self.fov = self.profile['fov']
        self.pixels_w = self.profile['pixels_w']
        self.pixels_h = self.profile['pixels_h']
        self.roi = self.profile['roi']
        self.sample_rate = self.profile['sample_rate']
        self.sensor_h = math.tan(self.fov / 2) * self.focal * 2
        self.intrinsics = self.get_intrinsics()
        self.extrinsics = self.get_extrinsics()

        self.roi_pids = list()
        self.fov_pids = list()


    def properties(self):
        properties = super().properties()
        properties.update(kind='Camera3D',focal=self.focal,
                          fov=self.fov,
                          pixels_w=self.pixels_w,
                          pixels_h=self.pixels_h,
                          sample_rate=self.sample_rate,
                          roi=self.roi,
                          intrinsics=self.get_intrinsics(),
                          extrinsics=self.get_extrinsics())
        return properties

    def restore(self):
        super().restore()
    
    def rtt(self):
        num_joints = p.getNumJoints(self.id)
        if num_joints: pos,orn,_,_,_,_ = p.getLinkState(self.id,num_joints-1)
        else: pos,orn = p.getBasePositionAndOrientation(self.id)
        pitch,roll,yaw = p.getEulerFromQuaternion(orn)

        near = self.focal
        far = 1000
        origin = np.array(pos)
        
        vm = p.computeViewMatrixFromYawPitchRoll(origin,near,math.degrees(yaw),math.degrees(pitch) - 90,math.degrees(roll),2)
        pm = p.computeProjectionMatrixFOV(math.degrees(self.fov),self.pixels_w/self.pixels_h,near,far)
        
        if self.roi_pids: self.draw_roi(False)
        _,_,pixels,depth_pixels,_ = p.getCameraImage(self.pixels_w,self.pixels_h,flags=-1,viewMatrix = vm,projectionMatrix = pm,renderer=p.ER_BULLET_HARDWARE_OPENGL)
        depth_pixels[:, :] = far * near / (far - (far - near) * depth_pixels[:, :]) - near
        if self.roi_pids: self.draw_roi(True)
        
        return pixels.tobytes(),depth_pixels.tobytes()

    def get_intrinsics(self):
        fx = self.pixels_w / 2 / math.tan(self.fov / 2)
        fy = self.pixels_h / 2 / math.tan(self.fov / 2)
        cx = self.pixels_w/2
        cy = self.pixels_h/2

        intrinsics = [
            [fy,0.0,cx],
            [0.0,fy,cy],
            [0.0,0.0,1.0],
        ]

        return intrinsics
    
    def get_extrinsics(self):
        extrinsics = np.identity(4)
        extrinsics[:3,:3] = Rotation.from_euler('xyz',self.rot).as_matrix()
        extrinsics[:3,3] = self.pos
        return extrinsics.tolist()
    
    def draw_fov(self,show=True):
        for pid in self.fov_pids: p.removeUserDebugItem(pid)
        self.fov_pids = list()
        if not show: return

        origin = np.array(self.pos)
        near = self.focal
        far = 1000
        ratio = self.pixels_w / self.pixels_h
        
        horizontal = np.array([self.sensor_h * ratio, 0, 0])
        vertical = np.array([0, self.sensor_h, 0])
        lower_left_corner = [0, 0, near] - horizontal/2 - vertical/2

        axes = [
            [1,0,0],
            [0,-1,0],
            [0,0,-1]]
        
        for j in range(2):
            for i in range(2):
                u = float(i) / (2-1)
                v = float(j) / (2-1)
                target_dr = Rotation.from_euler('xyz',self.rot).apply(lower_left_corner + u*horizontal + v*vertical) @ axes
                target = origin + target_dr
                l = np.linalg.norm(target - origin)
                dr = (target - origin) / l * far
                self.fov_pids.append(p.addUserDebugLine(origin, target + dr,[1,0,0],1,lifeTime=0))

    def draw_roi(self,show=True):
        for pid in self.roi_pids: p.removeUserDebugItem(pid)
        self.roi_pids = list()
        if not show: return
        x,y,z,rx,ry,rz,w,h,d = self.roi

        pos = np.array(self.pos)
        origin = np.array([x,y,z])
        size = np.array([w,h,d])
        radius = size / 2

        p0 = min = 0 - radius
        p1 = max = 0 + radius
        p2 = [max[0],min[1],min[2]]
        p3 = [min[0],max[1],min[2]]
        p4 = [min[0],min[1],max[2]]
        p5 = [min[0],max[1],max[2]]
        p6 = [max[0],min[1],max[2]]
        p7 = [max[0],max[1],min[2]]

        R = Rotation.from_euler('xyz',[rx,ry,rz])
        axes = [
            [-1,0,0],
            [0,1,0],
            [0,0,-1]]
        pn = pos + Rotation.from_euler('xyz',self.rot).apply(origin @ axes + R.apply([p0,p1,p2,p3,p4,p5,p6,p7]) @ axes)

        self.roi_pids = [
            p.addUserDebugLine(pn[0],pn[2],[1,1,0],1,lifeTime=0),
            p.addUserDebugLine(pn[0],pn[3],[1,1,0],1,lifeTime=0),
            p.addUserDebugLine(pn[0],pn[4],[1,1,0],1,lifeTime=0),
            p.addUserDebugLine(pn[1],pn[5],[1,1,0],1,lifeTime=0),
            p.addUserDebugLine(pn[1],pn[6],[1,1,0],1,lifeTime=0),
            p.addUserDebugLine(pn[1],pn[7],[1,1,0],1,lifeTime=0),
            p.addUserDebugLine(pn[2],pn[6],[1,1,0],1,lifeTime=0),
            p.addUserDebugLine(pn[2],pn[7],[1,1,0],1,lifeTime=0),
            p.addUserDebugLine(pn[3],pn[7],[1,1,0],1,lifeTime=0),
            p.addUserDebugLine(pn[4],pn[6],[1,1,0],1,lifeTime=0),
            p.addUserDebugLine(pn[5],pn[3],[1,1,0],1,lifeTime=0),
            p.addUserDebugLine(pn[5],pn[4],[1,1,0],1,lifeTime=0)
        ]
    
    def set_focal(self,focal):
        self.focal = focal
        self.fov = math.atan(self.sensor_h / 2 / self.focal) * 2

    def get_roi(self):
        return self.roi

    def set_roi(self,x,y,z,rx,ry,rz,w,h,d):
        self.roi = [x,y,z,rx,ry,rz,w,h,d]
        pass

    def signal_capture(self,*args,**kwargs):
        pixels,depth = self.rtt()
        self.result = (None,pixels,depth)

        try:
            sk = s.socket(s.AF_UNIX,s.SOCK_STREAM)
            sock_path = os.path.join(self.scene.tmp_dir,self.profile['name'] + '.sock')
            sk.connect(sock_path)
            for v in pixels,depth: sk.sendall(v)
        except: return
        finally:
            sk.close()

    def signal_pose_recognize(self,*args,**kwargs):
        pixels,depth = self.rtt()

        try:
            sk = s.socket(s.AF_UNIX,s.SOCK_STREAM)
            sock_path = os.path.join(self.scene.tmp_dir,self.profile['name'] + '.sock')
            sk.connect(sock_path)
            
            for v in pixels,depth: sk.sendall(v)
        except: 
            pass
        finally:
            sk.close()

        num_joints = p.getNumJoints(self.id)
        if num_joints: pos,orn,_,_,_,_ = p.getLinkState(self.id,num_joints-1)
        else: pos,orn = p.getBasePositionAndOrientation(self.id)

        far = 1000
        origin = np.array(pos)
        viewport_length = np.tan(self.fov / 2) * self.focal * 2

        horizontal = np.array([viewport_length, 0, 0])
        vertical = np.array([0, viewport_length, 0])
        lower_left_corner = [0, 0, -self.focal] - horizontal/2 - vertical/2
        ids = set()

        sample_rate = self.profile['sample_rate']
        for j in range(sample_rate-1,0,-1):
            for i in range(sample_rate):
                u = float(i) / (sample_rate-1)
                v = float(j) / (sample_rate-1)
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
               
                # axis_x = Rotation.from_quat(orn).apply(np.array([length,0,0])) + pos
                # axis_y = Rotation.from_quat(orn).apply(np.array([0,length,0])) + pos
                # axis_z = Rotation.from_quat(orn).apply(np.array([0,0,length])) + pos
                # p.addUserDebugLine(pos,axis_x,[1,0,0],2,lifeTime=0)
                # p.addUserDebugLine(pos,axis_y,[0,1,0],2,lifeTime=0)
                # p.addUserDebugLine(pos,axis_z,[0,0,1],2,lifeTime=0)
                val.append((pos,rot,None))
            self.result = (None,val) if val else ('failed',)
        self.actions.append((output, ()))

        
class Camera3DReal(ActiveObject):
    def __init__(self,scene,**kwargs):
        if 'focal' not in kwargs: kwargs['focal'] = 0.010
        if 'fov' not in kwargs: kwargs['fov'] = math.degrees(20)
        if 'pixels_w' not in kwargs: kwargs['pixels_w'] = 1024
        if 'pixels_h' not in kwargs: kwargs['pixels_h'] = 768
        if 'roi' not in kwargs: kwargs['roi'] = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        if 'sample_num' not in kwargs: kwargs['sample_num'] = 100000

        super().__init__(scene, **kwargs)
        self.focal = self.profile['focal']
        self.fov = self.profile['fov']
        self.pixels_w = self.profile['pixels_w']
        self.pixels_h = self.profile['pixels_h']
        self.roi = self.profile['roi']
        self.sample_num = self.profile['sample_num']
        self.sensor_h = np.tan(self.fov / 2) * self.focal * 2
        self.intrinsics = self.get_intrinsics()
        self.extrinsics = self.get_extrinsics()

        self.point_ids = list()
        self.rgb_pixels = bytes()
        self.depth_pixels = bytes()
    
    def properties(self):
        properties = super().properties()
        properties.update(kind='Camera3DReal',focal=self.focal,
                          fov=self.fov,
                          pixels_w=self.pixels_w,
                          pixels_h=self.pixels_h,
                          sample_rate=self.sample_num,
                          roi=self.roi,
                          intrinsics=self.get_intrinsics(),
                          extrinsics=self.get_extrinsics())
        return properties

    def restore(self):
        super().restore()
        self.clear_point_cloud()
        pass

    def rtt(self):
        self.signal_capture()
        
    def signal_check(self,*args,**kwargs):
        self.result = None,self.profile['eye_to_hand_transform']

    def signal_capture(self,*args,**kwargs):
        try:
            sk = s.socket(s.AF_UNIX,s.SOCK_STREAM)
            sock_path = os.path.join(self.scene.tmp_dir,self.profile['name'] + '.sock')
            sk.connect(sock_path)
            width = int.from_bytes(sk.recv(4),'little')
            height = int.from_bytes(sk.recv(4),'little')
            if not width or not height: raise 'failed'

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
        except: 
            self.result = 'failed',
            return
        finally:
            sk.close()

        self.clear_point_cloud()
        self.actions.append((self.draw_point_cloud_from_depth_pixels,(depth_pixels,rgb_pixels,width,height)))
        self.result = None,self.extrinsics
        
    def set_calibration(self,intrinsics,extrinsics):
        self.set_intrinsics(intrinsics)
        self.set_extrinsics(extrinsics)
        pass

    def get_intrinsics(self):
        fx = self.pixels_w / 2 / math.tan(self.fov / 2)
        fy = self.pixels_h / 2 / math.tan(self.fov / 2)
        cx = self.pixels_w/2
        cy = self.pixels_h/2

        intrinsics = [
            [fy,0.0,cx],
            [0.0,fy,cy],
            [0.0,0.0,1.0],
        ]
        return intrinsics

    def set_intrinsics(self,intrinsics):
        self.intrinsics = intrinsics
        m = np.array(intrinsics)

        fx = m[0,0]
        fy = m[1,1]
        cx = m[0,2]
        cy = m[1,2]

        self.pixels_w = cx * 2
        self.pixels_y = cy * 2
        self.fov = math.atan(self.pixels_w / 2 / fx) * 2
    
    def get_extrinsics(self):
        extrinsics = np.identity(4)
        extrinsics[:3,:3] = Rotation.from_euler('xyz',self.rot).as_matrix()
        extrinsics[:3,3] = self.pos
        return extrinsics.tolist()
    
    def set_extrinsics(self,extrinsics):
        self.extrinsics = extrinsics
        m = np.array(extrinsics)
        R = m[:3, :3]
        T = m[:3, 3]
        self.set_pos(T.tolist())
        self.set_rot(Rotation.from_matrix(R).as_euler('xyz').tolist())
        print(self.pos,self.rot)

    def draw_point_cloud_from_depth_pixels(self,depth_pixels,rgb_pixels,width,height):
        def depth_to_point_cloud(depth_image, intrinsics):
            rows, cols = depth_image.shape
            c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
            valid = (0 < depth_image) & (depth_image < 1000)
            z = np.where(valid, depth_image, np.nan)
            x = np.where(valid, z * (c - intrinsics[0, 2]) / intrinsics[0, 0], 0)
            y = np.where(valid, z * (r - intrinsics[1, 2]) / intrinsics[1, 1], 0)

            point_cloud = np.dstack((x, y, z))
            point_cloud = np.reshape(point_cloud, (rows * cols, 3))
            return point_cloud

        R = Rotation.from_euler('xyz',self.rot)
        T = self.pos
        sample = int(width*height / self.sample_num)
        sample = 1 if sample < 1 else sample
        
        pixels_size = 4 if len(rgb_pixels) == width * height * 4 else 3
        point_cloud_color = np.frombuffer(rgb_pixels, dtype=np.ubyte).reshape((width*height,pixels_size))[::sample] / 255
        depth_pixels = np.frombuffer(depth_pixels, dtype=np.float32).reshape((height,width))
        point_cloud = depth_to_point_cloud(depth_pixels, np.array(self.intrinsics) )[::sample]

        axes = [
            [1,0,0],
            [0,1,0],
            [0,0,1]]
        
        point_cloud = R.apply(point_cloud @ axes) + T 
        
        beg = 0; end = len(point_cloud)
        while beg < end:
            offset = end - beg
            if offset > 10000: offset = 10000
            point_id = p.addUserDebugPoints(point_cloud[beg:beg+offset],point_cloud_color[beg:beg+offset,0:3],1,lifeTime=0)
            self.point_ids.append(point_id)
            beg += offset


    def draw_point_cloud(self,vertexes,colors):
        R = Rotation.from_euler('xyz',self.rot)
        T = self.pos
        
        if len(vertexes) > len(colors): colors.extend([0,0,0]*(len(vertexes)-len(colors)))
        vertexes = R.apply(vertexes) + T
        
        beg = 0; end = len(vertexes)
        while beg < end:
            offset = end - beg
            if offset > 10000: offset = 10000
            point_id = p.addUserDebugPoints(vertexes[beg:beg+offset],colors[beg:beg+offset],1,lifeTime=0)
            self.point_ids.append(point_id)
        beg += offset
        pass

    def clear_point_cloud(self):
        for point_id in self.point_ids: p.removeUserDebugItem(point_id)
        pass
        
        