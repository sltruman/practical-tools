import pybullet as p

class ActiveObject:
    def __init__(self,scene,**kwargs):
        self.pos = kwargs['pos']
        self.rpy = kwargs['rpy']
        self.set_base(kwargs['base'])
        pass
    
    def update(self,dt):
        pass

    def set_base(self,base):
        self.base = base
        if 'id' in vars(self): p.removeBody(self.id)
        self.id = p.loadURDF(self.base, self.pos, p.getQuaternionFromEuler(self.rpy),useFixedBase=True)
        return self.id

    def properties(self):
        pos,orn = p.getBasePositionAndOrientation(self.id)
        rpy = p.getEulerFromQuaternion(orn)
        return dict(id=self.id,kind='ActiveObject',base=self.base,pos=pos,rpy=rpy)