import pybullet as p
import numpy as np
import random
import py3dbp as bp
import random
from .active_obj import ActiveObject

class Placer(ActiveObject):
    def __init__(self,scene,**kwargs):
        if 'workpiece_texture' not in kwargs: kwargs['workpiece_texture'] = '' 
        if 'workpiece' not in kwargs: kwargs['workpiece'] = 'workpieces/lego/lego.urdf' 
        if 'center' not in kwargs: kwargs['center'] = [0,0,0]
        if 'interval' not in kwargs: kwargs['interval'] = 1
        if 'amount' not in kwargs: kwargs['amount'] = 10
        super().__init__(scene,**kwargs)

        self.workpiece_texture = kwargs['workpiece_texture']
        self.workpiece = kwargs['workpiece']
        self.center = kwargs['center']
        self.interval = kwargs['interval']
        self.amount = kwargs['amount']

        self.objs = list()
        self.elapsed = 0
    
    def properties(self):
        properties = super().properties()
        properties.update(kind='Placer',workpiece=self.workpiece,center=self.center,interval=self.interval,amount=self.amount,workpiece_texture=self.workpiece_texture)
        return properties

    def update(self, dt):
        if not self.actions: return
        self.elapsed += dt
        if self.elapsed < self.profile['interval']: return
        self.elapsed = 0
        super().update(dt)

    def restore(self):
        super().restore()
        for obj_id in self.objs: p.removeBody(obj_id)
        self.objs.clear()

    def set_workpiece(self,base):
        self.workpiece = base

    def set_workpiece_texture(self,img_path):
        self.workpiece_texture = img_path

    def set_center(self,center):
        self.center = center

    def set_amount(self,num):
        self.amount = num

    def set_interval(self,seconds):
        self.interval = seconds

    def signal_generate(self,*args,**kwargs):
        def task():
            rot = np.array([random.randint(0,314),random.randint(0,314),random.randint(0,314)]) / 100.
            self.objs.append(p.loadURDF(self.profile['workpiece'],self.profile['center'],p.getQuaternionFromEuler(rot)))
        
        for i in range(self.profile['amount']): self.actions.append((task, ()))

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

    