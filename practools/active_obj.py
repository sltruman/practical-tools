import pybullet as p
import os
import pygfx as gfx

class ActiveObject(gfx.WorldObject):
    def __init__(self,data_dir='',**kwargs):
        self.actions = list()
        self.result = None,
        self.data_dir = data_dir
        self.profile = kwargs

        self.base = kwargs['base']
        self.pos = kwargs['pos']
        self.rot = kwargs['rot']
        self.user_data = self.profile['user_data'] if 'user_data' in self.profile else ''

        self.set_base(self.base)
        pass

    def __del__(self):
        self.actions.clear()
        if 'id' in vars(self): p.removeBody(self.id)
        pass
 
    def get_properties(self):
        return self.properties()

    def properties(self):
        return dict(kind=type(self),
                    id=self.id,
                    base=self.base,
                    pos=self.pos,
                    rot=self.rot,
                    links=self.get_links())
    
    def update(self):
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
        self.id = p.loadURDF(base, self.pos, p.getQuaternionFromEuler(self.rot),useFixedBase=True)
        p.resetBasePositionAndOrientation(self.id,self.pos,p.getQuaternionFromEuler(self.rot))
    
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