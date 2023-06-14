import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation
from time import time,sleep
import json
import os
import socket as s

class Scene:
  def __init__(self,width=1024,height=768,data_dir='./digitaltwin_data',tmp_dir='.'):
    self.tmp_dir = tmp_dir
    self.data_dir = data_dir

    self.tick = time()
    self.active_objs = dict()
    self.active_objs_by_name = dict()
    self.width = width
    self.height = height
    self.running = True
    self.scene_path = ''
    self.timestep = 1/180.
    self.actions = list()
    self.ground_z = 0
    
    self.id = p.connect(p.GUI,options=f'--width={width} --height={height} --headless')
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    p.setPhysicsEngineParameter(erp=1,contactERP=1,frictionERP=1)
    p.setTimeStep(self.timestep)
    p.setGravity(0, 0, -9.81)
    
    self.plane = p.loadURDF(os.path.join(self.data_dir,"pybullet_objects/plane.urdf"), [0, 0, self.ground_z], useFixedBase=True)

  def __del__(self):
    self.play(False)
    p.resetSimulation()
    del self.active_objs
    del self.active_objs_by_name
    pass

  def get_profile(self):
    return self.profile

  def load(self,scene_path):
    if not scene_path: return

    self.scene_path = scene_path
    with open(scene_path,'r') as f: self.profile = json.load(f)

    if 'ground_z' not in self.profile: self.profile['ground_z'] = 0
    if 'user_data' not in self.profile: self.profile['user_data'] = ''
    self.ground_z = self.profile['ground_z']
    self.user_data = self.profile['user_data']
    
    p.resetBasePositionAndOrientation(self.plane,[0,0,self.ground_z],p.getQuaternionFromEuler([0,0,0]))

    minimum=self.ground_z
    maximum=1.5
    for object_info in self.profile['active_objects']:
      # def task(object_info):
        print('add object:',object_info,flush=True)
        kind = object_info['kind']
        active_obj = None
        import digitaltwin
        active_obj = eval(f'digitaltwin.{kind}(self,**object_info)')
        self.active_objs[active_obj.id] = active_obj
        if 'name' in object_info: self.active_objs_by_name[object_info['name']] = active_obj

        lit,big = p.getAABB(active_obj.id)
        
        if lit[0] < minimum: minimum = lit[0]
        if lit[1] < minimum: minimum = lit[1]
        if lit[2] < minimum: minimum = lit[2]
        if big[0] > maximum: maximum = big[0]
        if big[1] > maximum: maximum = big[1]
        if big[2] > maximum: maximum = big[2]

    # self.actions.append((task,[object_info]))

    w,h,vm,pm,up,forward,horizontal,vertical,yaw,pitch,distance,target = p.getDebugVisualizerCamera()
    dist = maximum - minimum
    p.resetDebugVisualizerCamera(dist*1.2,45,-45,target)
    
  def save(self,scene_path=''):
      if scene_path: self.scene_path = scene_path
      if not self.scene_path: 
        print('failed to save and scene path is nothing.',flush=True)
        return

      self.profile['ground_z'] = self.ground_z
      self.profile['user_data'] = self.user_data
      active_objects = list()
      for obj in self.active_objs_by_name.values():
        obj.profile = obj.properties()
        active_objects.append(obj.profile)
      self.profile['active_objects'] = active_objects
      with open(self.scene_path,'w') as f: json.dump(self.profile,f,indent=2)

  def restore(self):
    for obj in self.active_objs_by_name.values():
      obj.restore()
    pass

  def rtt(self):
    _,_,pixels,_,_ = p.getCameraImage(self.width,self.height,flags=p.ER_NO_SEGMENTATION_MASK,renderer=p.ER_BULLET_HARDWARE_OPENGL)
    return pixels.tobytes()

  def play(self,run=True):
    self.running = run

  def update_for_tick(self,dt=1/180):
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

  def rotate(self,x,y):
    width,height,vm,pm,up,forward,horizontal,vertical,yaw,pitch,distance,target = p.getDebugVisualizerCamera()
    x,y=x*width,y*height
    yaw -= x

    pitch -= y / 2
    if pitch > -1:
      pitch = -1
    
    if pitch < -89:
      pitch = -89

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
    pitch = -1
    p.resetDebugVisualizerCamera(distance,yaw,pitch,target)
    pass

  def rotate_right(self):
    w,h,vm,pm,up,forward,horizontal,vertical,yaw,pitch,distance,target = p.getDebugVisualizerCamera()
    yaw = 89
    pitch = -1
    p.resetDebugVisualizerCamera(distance,yaw,pitch,target)
    pass
  
  def pan(self,x,y):
    width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, yaw,pitch, dist, camTarget = p.getDebugVisualizerCamera()
    x,y=x*width,y*height
    oneOverWidth = float(1) / float(width)
    oneOverHeight = float(1) / float(height)

    f = 10000 / dist
    sx = 20000 / f * oneOverWidth
    sy = 20000 / f * oneOverHeight

    forward = np.array([-float(x) * sx,float(y) * sy,0])
    pos = camTarget + Rotation.from_euler('xyz',[0,0,yaw],True).apply(forward)
    p.resetDebugVisualizerCamera(dist,yaw,pitch,pos)
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
  
  def set_ground_z(self,z):
    self.ground_z = z
    p.resetBasePositionAndOrientation(self.plane,[0,0,z],p.getQuaternionFromEuler([0,0,0]))
    pass

  def set_ground_texture(self,texture):
    self.ground_texture = texture
    p.changeVisualShape(self.plane,-1,textureUniqueId=p.loadTexture(texture))
    pass

  def set_user_data(self,val):
    self.user_data = val
