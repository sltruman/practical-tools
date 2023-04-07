import pybullet as p
import numpy as np
import random
import py3dbp as bp
import random
from .active_obj import ActiveObject

class Placer(ActiveObject):
    def __init__(self,scene,**kwargs):
        super().__init__(scene,**kwargs)
        self.objs = list()
        self.workpiece = kwargs['workpiece']
        self.center = kwargs['center']
        self.interval = kwargs['interval']
        self.amount = kwargs['amount']
        # self.workpiece_texture = kwargs['workpiece_texture']
        self.elapsed = 0
    def properties(self):
        info = super().properties()
        info.update(dict(kind='Packer',center=self.center,interval=self.interval,amount=self.amount,workpiece=self.workpiece))
        return info

    def update(self, dt):
        if not self.actions: return
        self.elapsed += dt
        if self.elapsed < self.interval: return
        self.elapsed = 0
        super().update(dt)

    def signal_generate(self,*args):
        def task():
            rot = np.array([random.randint(0,314),random.randint(0,314),random.randint(0,314)]) / 100.
            self.objs.append(p.loadURDF(self.workpiece,self.center,p.getQuaternionFromEuler(rot)))
        
        for i in range(self.amount): self.actions.append((task, ()))

        def task1():
            num = 0 
            for o in self.objs:
                linear,angular = p.getBaseVelocity(o)
                n = np.linalg.norm(linear) + np.linalg.norm(angular)
                if n > num: num = n
            if num > 0.08: self.actions.append((task1, ()))
        self.actions.append((task1, ()))
        def output(): self.result = (None,) if len(self.objs) < 100 else ('failed')
        self.actions.append((output, ()))
        pass

    