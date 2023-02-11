import pybullet as p
import numpy as np
import random
import py3dbp as bp

class Packer:
    def __init__(self,scene,**kwargs):
        self.scene = scene
        self.name = kwargs['name']
        self.base = kwargs['base']
        self.pos = np.array(kwargs['pos'])
        self.id = p.loadURDF(self.base, self.pos, useFixedBase=True)
        self.workpiece = kwargs['workpiece']
        self.actions = list()
        self.center = kwargs['center']

    def update(self, dt):
        if not self.actions:
           return

        f,args = self.actions.pop(0)
        f(*args)
        pass

    def idle(self):
        return 0 == len(self.actions)
    
    def generate(self):
        for i in range(10):
            self.id = p.loadURDF(self.workpiece, self.center)
            self.scene.update_for_tick(1)
        pass
    
    def properties(self):
        pos,orn = p.getBasePositionAndOrientation(self.id)
        rpy = p.getEulerFromQuaternion(orn)
        return dict(id=self.id,kind='Packer',base=self.base,pos=pos,rpy=rpy,size=self.size)