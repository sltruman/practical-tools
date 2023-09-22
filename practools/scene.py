import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation
from time import time,sleep
import json
import os

class Scene:
  def __init__(self,width=1024,height=768):
    self.active_objs = dict()
    self.active_objs_by_name = dict()
    self.width = width
    self.height = height
    self.viewport_size = [width,height]
    self.viewport_texture = bytes([0]*width*height*4)
    self.running = True
    self.scene_path = ''
    self.timestep = 1/180.
    self.actions = list()
    
    self.id = p.connect(p.DIRECT,options=f'--width={width} --height={height}')
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    p.setPhysicsEngineParameter(erp=1,contactERP=1,frictionERP=1)
    p.setTimeStep(self.timestep)
    p.setGravity(0, 0, -9.81)
    
  def __del__(self):
    self.play(False)
    p.resetSimulation()
    del self.active_objs
    del self.active_objs_by_name
    pass

  def get_profile(self):
    return self.profile

  def play(self,run=True):
    self.running = run

  def update(self):
    if not self.running: return

    if self.actions:
        fun,args = act = self.actions[0]
        fun(*args)
        self.actions.pop(0)

    for obj in self.active_objs.values():
      obj.update(self.timestep)
      
    p.stepSimulation()

  def get_active_obj_properties(self):
    objs = dict()
    for name,obj in self.active_objs_by_name.items():
      objs[name] = obj.properties()
    return objs
