import pybullet as p
import numpy as np
import random
import py3dbp as bp
from .active_obj import ActiveObject

class Stacker(ActiveObject):
    def __init__(self,scene,**kwargs):
        super().__init__(scene,**kwargs)
        self.objs = list()
        self.area = kwargs['area']
        self.pos = self.profile['pos']
        self.lower_left = self.pos - np.array([self.area[0],self.area[1],0]) / 2

        self.packer = bp.Packer()
        bin = bp.Bin('large-box', self.area[0],self.area[1],self.area[2], 70.0)
        self.packer.add_bin(bin)

        avgItemSize = kwargs['box_size']
        itemChangeSize = kwargs['random_factor']
        
        for idx in range(50):
            item_width = avgItemSize[0] + random.randint(-itemChangeSize[0]*100,itemChangeSize[0]*100) / 100 * avgItemSize[0] 
            item_height = avgItemSize[1] + random.randint(-itemChangeSize[1]*100,itemChangeSize[1]*100) / 100 * avgItemSize[1] 
            item_depth = avgItemSize[2] + random.randint(-itemChangeSize[2]*100,itemChangeSize[2]*100) / 100 * avgItemSize[2] 
            item_weight = 0.1
            self.packer.add_item(bp.Item('item'+str(idx), item_width, item_height, item_depth, item_weight))
        
        self.packer.pack(bigger_first=True)

    def update(self, dt):
        super().update(dt)

    def restore(self):
        super().restore()
        for obj_id in self.objs: p.removeBody(obj_id)
        self.objs.clear()

    def signal_generate(self):
        def task(item:bp.Item):
            item_width, item_height, item_depth = item.get_dimension()
            item_pos = self.lower_left + [float(item.position[0] + item_width / 2), float(item.position[1] + item_height / 2), float(item.position[2] + item_depth / 2)]
            item_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[item_width/2, item_height/2, item_depth/2])
            item_id = p.createMultiBody(item.weight, item_shape, -1, item_pos, p.getQuaternionFromEuler([0, 0, 0]))            
            self.objs.append(item_id)

        for item in self.packer.bins[0].bin_fitted_items:
            self.actions.append([task,(item,)])

        def output(): self.result = None,

        self.actions.append([output,()])
        pass
    