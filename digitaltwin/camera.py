import pybullet as p
from scipy.spatial.transform import Rotation
from .active_obj import ActiveObject

class Camera3D(ActiveObject):
    def __init__(self,scene,**kwargs):
        super().__init__(scene, **kwargs)
        self.image_size = kwargs['image_size']
        self.forcal = kwargs['forcal']
        self.fov = kwargs['fov']
        self.image_path = kwargs['image_path']
        self.sample_rate = kwargs['sample_rate']
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

        depth_img = np.zeros((self.image_size[1],self.image_size[0],3),np.uint8)
        depth_pixels.astype(np.uint8)
        
        for h in range(0, self.image_size[1]):
            for w in range(0, self.image_size[0]):
                v = far * near / (far - (far - near) * depth_pixels[h, w]) / depth_far * 255
                depth_img[h, w] = [v,v,v]

        if self.image_path:
            import cv2
            cv2.imwrite(self.image_path,depth_img)
        return pixels.tobytes(),depth_img.tobytes()

    def signal_capture(self,*args):
        def output(): 
            self.result = (None,) + self.rtt()

        self.actions.append((output, ()))
        pass

    def signal_pose_recognize(self,*args):
        import numpy as np
        num_joints = p.getNumJoints(self.id)
        pos,orn,_,_,_,_ = p.getLinkState(self.id,num_joints-1)

        near = self.forcal
        far = 1000
        origin = np.array(pos)
        viewport_length = np.tan(np.pi / 180 * self.fov / 2) * near * 2

        horizontal = np.array([viewport_length, 0, 0])
        vertical = np.array([0, 0, viewport_length])
        lower_left_corner = [0, near, 0] - horizontal/2 - vertical/2
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
            p.removeAllUserDebugItems()

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
        pass
    