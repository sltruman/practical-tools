import pygfx as gfx
import pybullet as bullet

import numpy as np
from scipy.spatial.transform import Rotation
from time import time,sleep

class Scene(gfx.Scene):
  actions = list()
  num_frame = 0

  def __init__(self):
    super().__init__()  
    id = bullet.connect(bullet.DIRECT)
    bullet.setPhysicsEngineParameter(erp=1,contactERP=1,frictionERP=1)
    bullet.setGravity(0, 0, -9.81)
    
  def __del__(self):
    # bullet.resetSimulation()
    pass

  def update(self):
    self.num_frame += 1

    if self.actions:
        fun,args = act = self.actions[0]
        fun(*args)
        self.actions.pop(0)

    for obj in self.active_objs.values():
      obj.update()

    bullet.stepSimulation()