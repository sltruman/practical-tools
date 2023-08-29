import pybullet as p
import os

class ActiveObject:
    def __init__(self,data_dir,**kwargs):
        self.actions = list()
        self.result = None,
        self.data_dir = data_dir
        self.profile = kwargs

        if 'name' in kwargs: self.name = kwargs['name']
        self.base = kwargs['base']
        self.pos = kwargs['pos']
        self.rot = kwargs['rot']
        self.user_data = self.profile['user_data'] if 'user_data' in self.profile else ''

        self.set_base(self.base)
        pass

    def remove(self):
        self.actions.clear()
        if 'id' in vars(self): p.removeBody(self.id)
        pass
 
    def get_properties(self):
        return self.properties()

    def properties(self):
        return dict(kind='ActiveObject',
                    name=self.name if 'name' in vars(self) else self.id,
                    base=self.base,
                    pos=self.pos,
                    rot=self.rot,
                    links=self.get_links(),
                    user_data=self.user_data if 'user_data' in vars(self) else '')
    
    def update(self,dt):
        self.pos,orn = p.getBasePositionAndOrientation(self.id)
        self.rot = p.getEulerFromQuaternion(orn)
        
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
        self.id = p.loadURDF(os.path.join(self.data_dir,base), self.pos, p.getQuaternionFromEuler(self.rot),useFixedBase=True)
        p.resetBasePositionAndOrientation(self.id,self.pos,p.getQuaternionFromEuler(self.rot))
        if 'name' not in vars(self): self.name = self.id
    
    def get_pose(self):
        return self.pos,self.rot
    
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

    def set_user_data(self,value):
        self.user_data = value

    def get_links(self):
        links = []
        shapes = p.getVisualShapeData(self.id)
        _,_,_,_,base,_,_,_ = shapes[0]
        pos,orn = p.getBasePositionAndOrientation(self.id)
        links.append(dict(base=base.decode(),pos=pos,rot=p.getEulerFromQuaternion(orn)))

        for i in range(p.getNumJoints(self.id)):
            _,_,_,_,pos,orn = p.getLinkState(self.id,i)
            _,_,_,_,base,_,_,_ = shapes[i+1]
            links.append(dict(base=base.decode(),pos=pos,rot=p.getEulerFromQuaternion(orn)))
        return links