import pybullet as p
import os
import pygfx as gfx

class ActiveObject(gfx.Mesh):
    def __init__(self,**kwargs):
        super().__init__()

        self.actions = list()
        self.result = None,

        self.base = kwargs['base']
        self.pos = kwargs['position']
        self.rot = kwargs['euler']

        self.set_base(self.base)
        pass

    def __del__(self):
        self.actions.clear()
        if 'urdf_id' in vars(self): p.removeBody(self.urdf_id)
        pass
 
    def get_properties(self):
        return self.properties()

    def properties(self):
        return dict(kind=type(self),
                    id=self.id,
                    base=self.base,
                    pos=self.pos,
                    rot=self.rot)
    
    def update(self):
        self.pos,orn = p.getBasePositionAndOrientation(self.urdf_id)
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
        if 'urdf_id' in vars(self): p.removeBody(self.urdf_id)

        visual = os.path.join(self.base,'visual.glb')
        physical = os.path.join(self.base,'physical.urdf')

        if os.path.exists(visual):
            meshes = gfx.load_meshes(visual)
            self.add(*meshes)
            
        if os.path.exists(physical):
            self.urdf_id = p.loadURDF(base, self.pos, p.getQuaternionFromEuler(self.rot),useFixedBase=True)

        p.resetBasePositionAndOrientation(self.id,self.pos,p.getQuaternionFromEuler(self.rot))

    def set_env_map(self,env_tex):
        for m in self.children:
            m.geometry.texcoords1 = m.geometry.texcoords
            m.material.env_map = env_tex
        pass
    
    def get_pose(self):
        return self.pos,self.rot
    
    def set_pos(self,pos):
        self.pos = pos
        p.resetBasePositionAndOrientation(self.urdf_id,pos,p.getQuaternionFromEuler(self.rot))
        pass

    def get_pos(self):
        return self.pos

    def set_rot(self,rot):
        self.rot = rot
        p.resetBasePositionAndOrientation(self.urdf_id,self.pos,p.getQuaternionFromEuler(rot))
        pass

    def get_rot(self):
        return self.rot
