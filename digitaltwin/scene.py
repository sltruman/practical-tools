import digitaltwin
import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation
from time import time,sleep
import json
import os

class Scene:
  def __init__(self,width=1024,height=768,tmp_dir='.'):
    self.tmp_dir = tmp_dir
    self.tick = time()
    self.active_objs = dict()
    self.active_objs_by_name = dict()
    self.viewport_size = int(width),int(height),4
    self.running = True
    self.scene_path = ''
    self.timestep = 1/180.
    self.actions = list()
    self.ground_z = 0
    
    self.id = p.connect(p.GUI,options=f'--width={width} --height={height} --headless')
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    p.setPhysicsEngineParameter(erp=1,contactERP=1,frictionERP=1)
    self.reset()

  def __del__(self):
    self.play(False)
    p.resetSimulation()
    del self.active_objs
    del self.active_objs_by_name
    pass

  def reset(self):
    self.play(False)
    p.resetSimulation()
    del self.active_objs
    del self.active_objs_by_name
    
    self.active_objs = dict()
    self.active_objs_by_name = dict()

    p.setGravity(0, 0, -9.81)
    p.setTimeStep(self.timestep)
    self.plane = p.loadURDF("./data/pybullet_objects/plane.urdf", [0, 0, self.ground_z], useFixedBase=True)
    self.load(self.scene_path)
    self.play(True)
    pass

  def load(self,scene_path):
    if not scene_path: return

    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    self.scene_path = scene_path
    with open(scene_path,'r') as f: self.profile = json.load(f)

    if 'ground_z' not in self.profile: self.profile['ground_z'] = 0
    self.plane = p.loadURDF("./data/pybullet_objects/plane.urdf", [0, 0, self.profile['ground_z']], useFixedBase=True)
    for object_info in self.profile['active_objects']:
      kind = object_info['kind']

      active_obj = None
      active_obj = eval(f'digitaltwin.{kind}(self,**object_info)')
      self.active_objs[active_obj.id] = active_obj
      if 'name' in vars(active_obj): self.active_objs_by_name[active_obj.name] = active_obj

  def save(self):
      self.scene.profile['active_objects']
      self.scene.profile['ground_z'] = self.ground_z
      active_objects = list()
      for obj in self.scene.active_objs_by_name.values():
        active_objects.append(obj.properties())
      self.scene.profile['active_objects'] = active_objects
      with open(self.scene_path,'w') as f: json.dump(f) 

  def rtt(self):
    _,_,pixels,_,_ = p.getCameraImage(self.viewport_size[0],self.viewport_size[1],renderer=p.ER_BULLET_HARDWARE_OPENGL)
    return pixels.tobytes(),

  def play(self,run=True):
    self.running = run
    pass

  def update_for_tick(self,dt):
    if not self.running: return

    if self.actions:
        fun,args = act = self.actions[0]
        fun(*args)
        self.actions.pop(0)

    while dt >= self.timestep:
      for obj in self.active_objs.values():
        obj.update(self.timestep)
        
      p.stepSimulation()
      dt -= self.timestep

  def update(self):
    if not self.running: return

    if self.actions:
      fun,args = act = self.actions[0]
      fun(*args)
      self.actions.pop(0)

    for obj in self.active_objs.values():
      obj.update(self.timestep)
    p.stepSimulation()

  def rotate(self,x,y):
    w,h,vm,pm,up,forward,horizontal,vertical,yaw,pitch,distance,target = p.getDebugVisualizerCamera()
    yaw -= 360 * x
    pitch -= 180 * y
    p.resetDebugVisualizerCamera(distance,yaw,pitch,target)
    pass

  def rotate_front(self):
    w,h,vm,pm,up,forward,horizontal,vertical,yaw,pitch,distance,target = p.getDebugVisualizerCamera()
    yaw = 0
    pitch = -1
    p.resetDebugVisualizerCamera(distance,yaw,pitch,target)
    pass

  def rotate_back(self):
    w,h,vm,pm,up,forward,horizontal,vertical,yaw,pitch,distance,target = p.getDebugVisualizerCamera()
    yaw = 180
    pitch = -1
    p.resetDebugVisualizerCamera(distance,yaw,pitch,target)
    pass

  def rotate_top(self):
    w,h,vm,pm,up,forward,horizontal,vertical,yaw,pitch,distance,target = p.getDebugVisualizerCamera()
    yaw = 0
    pitch = -89
    p.resetDebugVisualizerCamera(distance,yaw,pitch,target)
    pass

  def rotate_left(self):
    w,h,vm,pm,up,forward,horizontal,vertical,yaw,pitch,distance,target = p.getDebugVisualizerCamera()
    yaw = -89
    pitch = 0
    p.resetDebugVisualizerCamera(distance,yaw,pitch,target)
    pass

  def rotate_right(self):
    w,h,vm,pm,up,forward,horizontal,vertical,yaw,pitch,distance,target = p.getDebugVisualizerCamera()
    yaw = 89
    pitch = 0
    p.resetDebugVisualizerCamera(distance,yaw,pitch,target)
    pass
  
  def pan(self,x,y):
    w,h,vm,pm,up,forward,horizontal,vertical,yaw,pitch,distance,target = p.getDebugVisualizerCamera()
    length = np.array([-x * distance * 3,y * distance * 3,0])
    pos = target + Rotation.from_euler('xyz',[0,0,yaw],True).apply(length)
    p.resetDebugVisualizerCamera(distance,yaw,pitch,pos)
    pass

  def zoom(self,f):
    w,h,vm,pm,up,forward,horizontal,vertical,yaw,pitch,distance,target = p.getDebugVisualizerCamera()
    p.resetDebugVisualizerCamera(distance * f,yaw,pitch,target)
    pass
    
  def get_active_obj_properties(self):
    objs = dict()
    for name,obj in self.active_objs_by_name.items():
      objs[name] = obj.properties()
    return objs
  
