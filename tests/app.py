import pybullet_data as pd
import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation
import time
import asyncio
import random
from py3dbp import Bin,Item,Packer

p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setAdditionalSearchPath(pd.getDataPath())
p.setGravity(0, 0, -9.81)
# p.setRealTimeSimulation(1)

floor = p.loadURDF("models/floor/plane.urdf", [0, 0, 0], useFixedBase=True)

conveyor_forward1 = p.loadURDF("models/conveyor/robot.urdf", [0, 0.473806, 0], p.getQuaternionFromEuler([0,0,np.pi / 180 * -90]),useFixedBase=True)
conveyor_forward2 = p.loadURDF("models/conveyor/robot.urdf", [0.949238, 0.473806, 0], p.getQuaternionFromEuler([0,0,np.pi / 180 * -90]),useFixedBase=True)
conveyor_forward3 = p.loadURDF("models/conveyor/robot.urdf", [1.89503, 0.473806, 0], p.getQuaternionFromEuler([0,0,np.pi / 180 * -90]),useFixedBase=True)

for i in range(1,12):
  p.setJointMotorControl2(conveyor_forward1,i,p.VELOCITY_CONTROL,targetVelocity=10)
  p.setJointMotorControl2(conveyor_forward2,i,p.VELOCITY_CONTROL,targetVelocity=10)
  p.setJointMotorControl2(conveyor_forward3,i,p.VELOCITY_CONTROL,targetVelocity=10)

robot_in = p.loadURDF("models/robots/urdf/ur5.urdf",[0,0,0], useFixedBase=True)
robot_forward = p.loadURDF("models/robots/urdf/ur5.urdf",[2 ,1  ,0], useFixedBase=True)
rp=[ 1.57, 0, 0, -1.57, 1.57, 0, 0, 0]
for j in range(p.getNumJoints(robot_forward)):
  p.resetJointState(robot_forward, j, rp[j])
  p.resetJointState(robot_in, j, rp[j])

def escaped_time() -> float:
  global tick
  if 'tick' not in globals():
    tick = time.time()

  dt,tick = time.time() - tick,time.time()
  return dt

def escaped_time_reset():
  global tick
  del tick

async def packet_in():
  print('传送物料中...')

  bin_pos = np.array([-0.249507,-0.842426,0.01])
  bin_size = np.array([0.5, 0.5, 0.1])
  packer = Packer()

  bin = Bin('large-box', bin_size[0],bin_size[1],bin_size[2], 70.0)
  packer.add_bin(bin)

  p.addUserDebugLine(bin_pos,bin_pos + [bin_size[0],0,0],[1,0,0],lineWidth=10,lifeTime=0)
  p.addUserDebugLine(bin_pos,bin_pos + [0,bin_size[1],0],[0,1,0],lineWidth=10,lifeTime=0)
  p.addUserDebugLine(bin_pos,bin_pos + [0,0,bin_size[2]],[0,0,1],lineWidth=10,lifeTime=0)

  avgItemSize = [0.1, 0.1, 0.1]
  itemChangeSize = [0.0, 0.0, 0.0]
  for idx in range(200):
      item_width = avgItemSize[0] + 2 * itemChangeSize[0] * (random.random() - 0.5)
      item_height = avgItemSize[1] + 2 * itemChangeSize[1] * (random.random() - 0.5)
      item_depth = avgItemSize[2] + 2 * itemChangeSize[2] * (random.random() - 0.5)
      item_weight = 0.1

      packer.add_item(Item('item'+str(idx), item_width, item_height, item_depth, item_weight))

  packer.pack(bigger_first=True)

  for idx, item in enumerate(packer.bins[0].bin_fitted_items):
    item_width, item_height, item_depth = item.get_dimension()
    item_pos = bin_pos + [float(item.position[0] + item_width / 2), float(item.position[1] + item_height / 2), float(item.position[2] + item_depth / 2)]
    item_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[item_width/2, item_height/2, item_depth/2])
    item_id = p.createMultiBody(item.weight, item_shape, -1, item_pos, p.getQuaternionFromEuler([0, 0, 0]))
  
  await asyncio.sleep(0.05)
  asyncio.create_task(vision_detect_for_grip())
  pass

async def vision_detect_for_grip():
  print('视觉检测中')
  
  origin = np.array([0.,-0.6,0.7])

  aspect_ratio = 1 / 1
  image_width = 10
  image_height = int(image_width / aspect_ratio)

  viewport_height = 1
  viewport_width = aspect_ratio * viewport_height
  focal_length = 1

  horizontal = np.array([viewport_width, 0, 0])
  vertical = np.array([0, viewport_height, 0])
  lower_left_corner = origin - horizontal/2 - vertical/2 - np.array([0, 0, focal_length])

  rays = list()
  for j in range(image_height-1,0,-1):
    for i in range(image_width):
      u = float(i) / (image_width-1)
      v = float(j) / (image_height-1)
      target = lower_left_corner + u*horizontal + v*vertical
      rays.append(target)
      await asyncio.sleep(1/240.)
  
  global floor
  p.setCollisionFilterGroupMask(floor,-1,0x7fffffff,0x7ffffff0)
  intersections = p.rayTestBatch([origin]*len(rays),rays,numThreads=len(rays),collisionFilterMask=0x1)
  objs = list()
  for i,intersection in enumerate(intersections): 
    p.addUserDebugLine(origin,rays[i],[1,0,0],lifeTime=0.1)
    id = intersection[0]

    if id == -1 :
      continue

    pos = np.array(intersection[3])
    normal = np.array(intersection[4])
    objs.append([id,pos,normal])
    p.addUserDebugLine(pos, pos + normal * 0.2,[0,0,1],lifeTime=0.1)

  if objs:
    print('检测到物体，开始抓取')
    asyncio.create_task(route_to_grip(objs))
    return
  
  print('没有检测到物体')
  asyncio.create_task(packet_in())
  pass

async def route_to_grip(objs):
  obj,pos,normal = objs[0]
  aabbMin,aabbMax = p.getAABB(obj)
  obj_pos,obj_orn = p.getBasePositionAndOrientation(obj)
  gripped_pos = np.array(obj_pos) + [0,0,abs(aabbMax[2] - aabbMin[2]) / 2.0]
  gripped_orn = p.getQuaternionFromEuler([np.pi,0,0])

  p.addUserDebugLine(aabbMin,aabbMax,lifeTime=0)
  p.addUserDebugLine(aabbMin,aabbMax,lifeTime=0)
  p.addUserDebugLine(aabbMin,aabbMax,lifeTime=0)

  paths = list()
  paths.append([gripped_pos + [0,0,0.2],gripped_orn, None])
  paths.append([gripped_pos, gripped_orn, obj])
  paths.append([gripped_pos + [0,0,0.2],gripped_orn, None])

  for path in paths:
    p.addUserDebugPoints([path[0]],[[1,0,0]],pointSize=10,lifeTime=0)

  asyncio.create_task(grip(paths))
  pass

forces = dict()
async def grip(paths : list):
  dt = escaped_time()

  target_pos,target_orn,obj = paths[0]
  effector_pos = move(robot_in,7,target_pos,target_orn,dt)
  length = np.linalg.norm(effector_pos - target_pos)
  await asyncio.sleep(0.05)

  if length < 0.001: # 吸附
    paths.pop(0)

    if obj:
      closest_points = p.getClosestPoints(robot_in,obj,0.02,7)
      print('xx',closest_points)
      if not closest_points:
        loop.create_task(grip(paths))
        return

      for _,_,_,_,_,effector_pos,obj_pos,_,_,_,_,_,_,_ in closest_points:
        p.addUserDebugLine(effector_pos,obj_pos,lifeTime=0)

      forces[obj] = (absorb,(obj,np.array(obj_pos) - p.getBasePositionAndOrientation(obj)[0]))

    if not paths:
      loop.create_task(route_to_place(obj))
      return

  loop.create_task(grip(paths))
  pass

def absorb(*args):
  obj,pos = args
  obj_pos = p.getBasePositionAndOrientation(obj)[0]

  effector_state = p.getLinkState(robot_in,7)
  effector_pos = np.array(effector_state[0])

  obj_pos = np.array(obj_pos) + pos
  effector_pos = np.array(effector_pos)

  obj_info = p.getDynamicsInfo(obj,-1)
  obj_mass = obj_info[0]

  direction = effector_pos - obj_pos
  direction = direction / np.linalg.norm(direction)

  p.addUserDebugLine(obj_pos,obj_pos + direction * obj_mass,lifeTime=0.1)

  p.applyExternalForce(obj,-1,direction * 20 * obj_mass * 9.81 ,obj_pos,p.WORLD_FRAME)

  pass

async def route_to_place(obj):
  placed_pos = np.array([-0.326556,0.455174,0.30])
  placed_orn = p.getQuaternionFromEuler([np.pi,0,0])

  paths = list()
  paths.append(([-0.322221 ,0.164699 ,0.599546 ],placed_orn,None))
  paths.append((placed_pos,placed_orn,obj))
  paths.append(([-0.322221 ,0.164699 ,0.599546 ],placed_orn,None))

  for path in paths:
    p.addUserDebugPoints([path[0]],[[1,0,0]],pointSize=10,lifeTime=0)

  loop.create_task(place(paths))
  pass

async def place(paths : list):
  dt = escaped_time()
  target_pos,target_orn,obj = paths[0]
  effector_pos = move(robot_in,7,target_pos,target_orn,dt)
  if np.linalg.norm(effector_pos - target_pos) < 0.001: # 分离
    paths.pop(0)
    
    if obj: del forces[obj]

    if not paths:
      loop.create_task(vision_detect_for_grip())
      escaped_time_reset()
      return


  await asyncio.sleep(0.05)
  loop.create_task(place(paths))
  pass

def move(robot,effector_index,target_pos,target_orn,dt):
  effector_state = p.getLinkState(robot,effector_index)
  effector_pos = np.array(effector_state[0])

  direction = target_pos - effector_pos
  length = np.linalg.norm(direction)
  if length < dt: 
    increment_direction = direction
  else: 
    increment_direction = direction / length * dt
  effector_pos += increment_direction

  poses = p.calculateInverseKinematics(robot_in, effector_index, effector_pos, target_orn,jointDamping=[ 0, 0.9, 0, 0, 0, 0, 0, 0], maxNumIterations=200)
  for i in range(6): p.setJointMotorControl2(robot_in, i, p.POSITION_CONTROL, poses[i])

  return effector_pos

def move_for_curve(robot,effector_index,target_pos,target_orn,dt):
  origin_pos = np.array(p.getBasePositionAndOrientation(robot)[0])
  effector_state = p.getLinkState(robot,effector_index)
  effector_pos = np.array(effector_state[0])

  effector_direction = effector_pos - origin_pos
  target_direction = target_pos - origin_pos
  effector_direction_length = np.linalg.norm(effector_direction) 
  target_direction_length = np.linalg.norm(target_direction)

  angle_radian=np.arccos(np.dot(effector_direction,target_direction) / (np.linalg.norm(effector_direction) * np.linalg.norm(target_direction))) #旋转弧度
  angular_velocity = dt / effector_direction_length
  if angle_radian < angular_velocity: 
    angular_velocity = angle_radian

  rotated_axis = np.cross(effector_direction,target_direction) # 计算最佳转向轴
  if np.linalg.norm(rotated_axis):
    rotated_axis = rotated_axis / np.linalg.norm(rotated_axis)

  r = Rotation.from_rotvec(angular_velocity * rotated_axis)

  diff_length = target_direction_length - effector_direction_length
  if abs(diff_length) < dt:
    increment_direction = effector_direction * (diff_length / effector_direction_length)
  else: 
    increment_direction = effector_direction * (diff_length / abs(diff_length) * dt / effector_direction_length)
  effector_direction += increment_direction

  effector_pos = origin_pos + r.apply(effector_direction)

  # print('差',diff_length,'角度',np.rad2deg(angle_radian),'角速:',np.rad2deg(angular_velocity))

  poses = p.calculateInverseKinematics(robot_in, effector_index, effector_pos, target_orn,jointDamping=[ 0, 0.9, 0, 0, 0, 0, 0, 0], maxNumIterations=200)
  for i in range(6): p.setJointMotorControl2(robot_in, i, p.POSITION_CONTROL, poses[i])
  return effector_pos



async def step_simulation():
  for f,args in forces.values():
    f(*args)

  p.stepSimulation()
  await asyncio.sleep(1/240.)
  
loop = asyncio.get_event_loop()
loop.create_task(step_simulation())
loop.run_forever()