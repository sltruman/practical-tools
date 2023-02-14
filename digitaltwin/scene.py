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

    self.id = p.connect(p.GUI,options=f'--width={width} --height={height} --headless')
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

  def __del__(self):
    p.resetSimulation()
    p.disconnect()
    pass

  def reset(self):
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    self.plane = p.loadURDF("./data/pybullet_data/plane.urdf", [0, 0, 0], useFixedBase=True)
    self.load(self.scene_path)
    pass

  def load(self,scene_path):
    if scene_path:
      pass

    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    self.plane = p.loadURDF("./data/pybullet_data/plane.urdf", [0, 0, 0], useFixedBase=True)

    self.scene_path = scene_path
    with open(scene_path,'r') as f:
      self.profile = scene_info = json.load(f)

    from digitaltwin import Robot,Camera3D,Packer,Stacker
        
    for object_info in scene_info['active_objects']:
      print(object_info)
      kind = object_info['kind']

      active_obj = None
      active_obj = eval(f'{kind}(self,**object_info)')
      self.active_objs[active_obj.id] = active_obj
      if 'name' in vars(active_obj): self.active_objs_by_name[active_obj.name] = active_obj


  def rtt(self):
    _,_,pixels,_,_ = p.getCameraImage(self.viewport_size[0],self.viewport_size[1],renderer=p.ER_BULLET_HARDWARE_OPENGL)
    return pixels.tobytes()

  def play(self,run=True):
    self.running = run
    pass

  def update(self):
    if not self.running: return

    dt = time() - self.tick
    while dt >= 1./240:
      for obj in self.active_objs.values():
        obj.update(1./240)
      p.stepSimulation()
      dt -= 1./240
      self.tick = time()

  def update_for_tick(self,dt):
    if not self.running: return
    
    while dt >= 1./240:
      for obj in self.active_objs.values():
        obj.update(1./240)
      p.stepSimulation()
      dt -= 1./240

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
