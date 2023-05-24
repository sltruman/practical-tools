import pybullet as p
import numpy as np
import random
import py3dbp as bp
from .active_obj import ActiveObject

class Stacker(ActiveObject):
    def __init__(self,scene,**kwargs):
        if 'workpiece_texture' not in kwargs: kwargs['workpiece_texture'] = '' 
        if 'area' not in kwargs: kwargs['area'] = [0.3,0.3,0.3]
        if 'box_size' not in kwargs: kwargs['box_size'] = [0.1,0.1,0.05]
        if 'random_factor' not in kwargs: kwargs['random_factor'] = [0.2,0.2,0.0]

        super().__init__(scene,**kwargs)

        self.workpiece_texture = kwargs['workpiece_texture']
        self.area = kwargs['area']
        self.box_size = kwargs['box_size']
        self.random_factor = kwargs['random_factor']

        self.objs = list()
        self.lower_left = self.pos - np.array([self.area[0],self.area[1],0]) / 2

        self.packer = bp.Packer()
        bin = bp.Bin('large-box', self.area[0],self.area[1],self.area[2], 70.0)
        self.packer.add_bin(bin)

        for idx in range(50):
            item_width = self.box_size[0] + random.randint(-self.random_factor[0]*100,self.random_factor[0]*100) / 100 * self.box_size[0] 
            item_height = self.box_size[1] + random.randint(-self.random_factor[1]*100,self.random_factor[1]*100) / 100 * self.box_size[1] 
            item_depth = self.box_size[2] + random.randint(-self.random_factor[2]*100,self.random_factor[2]*100) / 100 * self.box_size[2] 
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
    