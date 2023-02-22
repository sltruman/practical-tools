import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation
from time import time
import json

class Scene:
  def __init__(self,width=1024,height=768):
    self.tick = time()
    self.active_objs = dict()
    self.active_objs_by_name = dict()
    self.viewport_size = int(width),int(height),4
    self.running = True
    self.scene_path = ''
    self.timestep = 1/180.
    
    self.id = p.connect(p.GUI,options=f'--width={width} --height={height} --headless')
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
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
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(self.timestep)
    self.plane = p.loadURDF("./data/pybullet_objects/plane.urdf", [0, 0, 0], useFixedBase=True)
    self.load(self.scene_path)
    self.play(True)
    pass

  def load(self,scene_path):
    if not scene_path: return

    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    self.plane = p.loadURDF("./data/pybullet_objects/plane.urdf", [0, 0, 0], useFixedBase=True)

    self.scene_path = scene_path
    with open(scene_path,'r') as f: self.profile = scene_info = json.load(f)

    from digitaltwin import Robot,Camera3D,Placer,Stacker
        
    for object_info in scene_info['active_objects']:
      print(object_info)
      kind = object_info['kind']

      active_obj = None
      active_obj = eval(f'{kind}(self,**object_info)')
      self.active_objs[active_obj.id] = active_obj
      if 'name' in vars(active_obj): self.active_objs_by_name[active_obj.name] = active_obj

  def rtt(self):
    _,_,pixels,_,_ = p.getCameraImage(self.viewport_size[0],self.viewport_size[1],renderer=p.ER_BULLET_HARDWARE_OPENGL)
    return pixels.tobytes(),

  def play(self,run=True):
    self.running = run
    pass

  def update_for_tick(self,dt):
    if not self.running: return

    while dt >= self.timestep:
      for obj in self.active_objs.values():
        obj.update(self.timestep)
      p.stepSimulation()
      dt -= self.timestep

  def update(self):
    if not self.running: return

    for obj in self.active_objs.values():
      obj.update(self.timestep)
    p.stepSimulation()

  def rotate(self,x,y):
      w,h,vm,pm,up,forward,horizontal,vertical,yaw,pitch,distance,target = p.getDebugVisualizerCamera()
      yaw -= 360 * x
      pitch -= 180 * y
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
