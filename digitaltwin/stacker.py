import pybullet as p
import numpy as np
import random
import py3dbp as bp

class Stacker:
    def __init__(self,scene,**kwargs):
        self.base = kwargs['base']
        self.pos = np.array(kwargs['pos'])
        self.id = p.loadURDF(self.base, self.pos, useFixedBase=True)

        self.action = list()
        self.size = kwargs['size']

        self.packer = bp.Packer()
        bin = bp.Bin('large-box', self.size[0],self.size[1],self.size[2], 70.0)
        self.packer.add_bin(bin)

        p.addUserDebugLine(self.pos,self.pos + [self.size[0],0,0],[1,0,0],lineWidth=1,lifeTime=0)
        p.addUserDebugLine(self.pos,self.pos + [0,self.size[1],0],[0,1,0],lineWidth=1,lifeTime=0)
        p.addUserDebugLine(self.pos,self.pos + [0,0,self.size[2]],[0,0,1],lineWidth=1,lifeTime=0)

        avgItemSize = [0.1, 0.1, 0.1]
        itemChangeSize = [0.0, 0.0, 0.0]
        for idx in range(200):
            item_width = avgItemSize[0] + 2 * itemChangeSize[0] * (random.random() - 0.5)
            item_height = avgItemSize[1] + 2 * itemChangeSize[1] * (random.random() - 0.5)
            item_depth = avgItemSize[2] + 2 * itemChangeSize[2] * (random.random() - 0.5)
            item_weight = 0.1
            self.packer.add_item(bp.Item('item'+str(idx), item_width, item_height, item_depth, item_weight))
        
        self.packer.pack(bigger_first=True)

    def update(self, dt):
        if not self.action:
           return

        f,args = self.action.pop(0)
        f(*args)
        pass
    
    def generate(self):
        self.action.append([self.run,()])
        pass
    
    def run(self):
        for item in self.packer.bins[0].bin_fitted_items:
            item_width, item_height, item_depth = item.get_dimension()
            item_pos = self.pos + [float(item.position[0] + item_width / 2), float(item.position[1] + item_height / 2), float(item.position[2] + item_depth / 2)]
            item_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[item_width/2, item_height/2, item_depth/2])
            item_id = p.createMultiBody(item.weight, item_shape, -1, item_pos, p.getQuaternionFromEuler([0, 0, 0]))

    def properties(self):
        pos,orn = p.getBasePositionAndOrientation(self.id)
        rpy = p.getEulerFromQuaternion(orn)
        return dict(id=self.id,kind='Packer',base=self.base,pos=pos,rpy=rpy,size=self.size)