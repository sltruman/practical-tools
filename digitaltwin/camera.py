import pybullet as p
from scipy.spatial.transform import Rotation

class Camera3D:
    def __init__(self,scene,**kwargs):
        self.name = kwargs['name']
        self.pos = kwargs['pos']
        self.rpy = kwargs['rpy']
        self.set_base(kwargs['base'])
        self.image_size = kwargs['image_size']
        self.forcal = kwargs['forcal']
        self.fov = kwargs['fov']
        pass
    
    def update(self,dt):
        pass

    def set_base(self,base):
        self.base = base
        if 'id' in vars(self): p.removeBody(self.id)
        self.id = p.loadURDF(self.base, self.pos, p.getQuaternionFromEuler(self.rpy),useFixedBase=True)
        return self.id

    def properties(self):
        pos, orn = p.getBasePositionAndOrientation(self.id)
        rpy = p.getEulerFromQuaternion(orn)
        return dict(id=self.id,kind='Camera3D', base=self.base,pos=pos,rpy=rpy,image_size=self.image_size,fov=self.fov,forcal=self.forcal)
    
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
        sample_rate = 5
        for j in range(sample_rate-1,0,-1):
            for i in range(sample_rate):
                u = float(i) / (sample_rate-1)
                v = float(j) / (sample_rate-1)
                target = lower_left_corner + u*horizontal + v*vertical
                target = origin + Rotation.from_quat(orn).apply(target)
                
                l = np.linalg.norm(target - origin)
                dr = (target - origin) / l * far

                p.addUserDebugLine(origin, target + dr,[1,0,0],1,lifeTime=5)
                rayInfo = p.rayTest(origin, target + dr)
                if not rayInfo: continue
                
                id,linkindex,fraction,pos,norm = rayInfo[0]
                distance = np.linalg.norm(pos - origin)
                if depth_far < distance: depth_far = distance
        depth_far -= near

        
        vm = p.computeViewMatrixFromYawPitchRoll(origin,near,180 / np.pi * yaw,180 / np.pi * pitch,180 / np.pi * roll,2)
        pm = p.computeProjectionMatrixFOV(self.fov,self.image_size[0]/self.image_size[1],near,far)
        _,_,pixels,depth_pixels,_ = p.getCameraImage(self.image_size[0],self.image_size[1],viewMatrix = vm,projectionMatrix = pm,renderer=p.ER_BULLET_HARDWARE_OPENGL)

        depth_img = np.zeros((self.image_size[1],self.image_size[0],3),np.uint8)
        depth_pixels.astype(np.uint8)
        
        for h in range(0, self.image_size[1]):
            for w in range(0, self.image_size[0]):
                v = far * near / (far - (far - near) * depth_pixels[h, w]) / depth_far * 255
                depth_img[h, w] = [v,v,v]

        return pixels.tobytes() + depth_img.tobytes()

    def result(self):
        pass

    