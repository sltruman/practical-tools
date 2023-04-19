import pybullet as p
import os

class ActiveObject:
    def __init__(self,scene,**kwargs):
        self.actions = list()
        self.result = None,
        self.scene = scene
        self.profile = kwargs
        if 'scale' not in kwargs: self.profile['scale'] = [1,1,1] 
        
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
        self.profile['base'] = base
        if 'id' in vars(self): p.removeBody(self.id)
        self.id = p.loadURDF(base, self.profile['pos'], p.getQuaternionFromEuler(self.profile['rot']),useFixedBase=True)

    def set_pos(self,pos):
        self.profile['pos'] = pos
        p.resetBasePositionAndOrientation(self.id,pos,p.getQuaternionFromEuler(self.profile['rot']))
        pass

    def get_pos(self):
        return self.profile['pos']

    def set_rot(self,rot):
        self.profile['rot'] = rot
        p.resetBasePositionAndOrientation(self.id,self.profile['pos'],p.getQuaternionFromEuler(rot))
        pass

    def get_rot(self):
        return self.profile['rot']

    def set_scale(self,scale):
        pass

    def get_scale(self,):
        return [1,1,1]
    
    def set_transparence(self,value):
        pass

    def get_transparence(self):
        pass 

    def set_user_data(self,value):
        self.profile['user_data'] = value