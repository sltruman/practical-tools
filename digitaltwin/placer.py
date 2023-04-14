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

        if 'workpiece' not in kwargs: self.profile['workpiece'] = 'data/workpieces/lego.urdf' 
        if 'center' not in kwargs: self.profile['center'] = [0,0,0]
        if 'interval' not in kwargs: self.profile['interval'] = 1
        if 'amount' not in kwargs: self.profile['amount'] = 10
        # self.workpiece_texture = kwargs['workpiece_texture']
        self.elapsed = 0

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
        self.profile['workpiece'] = base

    def set_workpiece_texture(self,img_path):
        self.profile['workpiece_texture'] = img_path

    def set_center(self,center):
        self.profile['center'] = center

    def set_amount(self,num):
        self.profile['amount'] = num

    def set_interval(self,seconds):
        self.profile['interval'] = seconds

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

    