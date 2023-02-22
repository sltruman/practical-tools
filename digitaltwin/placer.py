import pybullet as p
import numpy as np
import random
import py3dbp as bp
import random
from .active_obj import ActiveObject

class Placer(ActiveObject):
    def __init__(self,scene,**kwargs):
        super().__init__(scene,**kwargs)
        self.workpiece = kwargs['workpiece']
        self.center = kwargs['center']
        self.interval = kwargs['interval']
        self.amount = kwargs['amount']
        self.number = 0
        self.elapsed = 0

    def properties(self):
        info = super().properties()
        info.update(dict(kind='Packer',center=self.center,interval=self.interval,amount=self.amount))
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
            p.loadURDF(self.workpiece,self.center,p.getQuaternionFromEuler(rot))
            self.number += 1
            
        for i in range(self.amount):
            self.actions.append((task, ()))

        def output(): 
            self.result = (None,) if self.number < 100 else ('out of amount')
        self.actions.append((output, ()))
        pass
