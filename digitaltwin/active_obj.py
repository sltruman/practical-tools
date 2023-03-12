import pybullet as p

class ActiveObject:
    def __init__(self,scene,**kwargs):
        self.actions = list()
        self.result = None,
        self.scene = scene
        self.name = kwargs['name']
        self.pos = kwargs['pos']
        self.rot = kwargs['rot']
        self.set_base(kwargs['base'])
        pass
    
    def properties(self):
        pos,orn = p.getBasePositionAndOrientation(self.id)
        rot = p.getEulerFromQuaternion(orn)
        return dict(id=self.id,kind='ActiveObject',base=self.base,pos=pos,rot=rot)
    
    def update(self,dt):
        if not self.actions: return
        fun,args = act = self.actions[0]
        fun(*args)
        self.actions.pop(0)
        pass

    def idle(self):
        return 0 == len(self.actions)

    def set_base(self,base):
        self.base = base
        if 'id' in vars(self): p.removeBody(self.id)
        self.id = p.loadURDF(self.base, self.pos, p.getQuaternionFromEuler(self.rot),useFixedBase=True,flags=p.URDF_ENABLE_SLEEPING)
        return self.id
