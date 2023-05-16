import pybullet as p
import os

class ActiveObject:
    def __init__(self,scene,**kwargs):
        if 'scale' not in kwargs: kwargs['scale'] = [1,1,1] 

        self.actions = list()
        self.result = None,
        self.scene = scene
        self.profile = kwargs

        self.base = kwargs['base']
        self.pos = kwargs['pos']
        self.rot = kwargs['rot']
        self.scale = kwargs['scale']
        self.set_base(kwargs['base'])
        pass

    def __del__(self):
        p.removeBody(self.id)
        pass

    def properties(self):
        return self.profile
    
    def update(self,dt):
        if not self.actions: return
        fun,args = self.actions[0]
        fun(*args)
        self.actions.pop(0)
        pass

    def idle(self):
         return 0 == len(self.actions)

    def restore(self):
        self.actions.clear()
        pass

    def set_base(self,base):
        self.base = base
        if 'id' in vars(self): p.removeBody(self.id)
        self.id = p.loadURDF(os.path.join(self.scene.data_dir,base), self.pos, p.getQuaternionFromEuler(self.rot),useFixedBase=True)

    def set_pos(self,pos):
        self.pos = pos
        p.resetBasePositionAndOrientation(self.id,pos,p.getQuaternionFromEuler(self.rot))
        pass

    def get_pos(self):
        return self.pos

    def set_rot(self,rot):
        self.rot = rot
        p.resetBasePositionAndOrientation(self.id,self.pos,p.getQuaternionFromEuler(rot))
        pass

    def get_rot(self):
        return self.rot

    def set_scale(self,scale):
        pass

    def get_scale(self,):
        return [1,1,1]
    
    def set_transparence(self,value):
        pass

    def get_transparence(self):
        pass 

    def set_user_data(self,value):
        self.user_data = value
