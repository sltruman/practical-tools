import pybullet_data as pd
import pybullet as p
import numpy as np
from threading import Thread, Timer
import time
import random
import math
p.connect(p.GUI) # or p.DIRECT for non-graphical version 
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
# p.setAdditionalSearchPath(pd.getDataPath())
p.setAdditionalSearchPath('models') #optionally 
p.setGravity(0, 0, -9.81)

floor = p.loadURDF("floor/plane.urdf", [0, 0, 0])
panda = p.loadURDF("franka_panda/panda.urdf",[0,0,0], useFixedBase=True)
tray = p.loadURDF("tray/traybox.urdf", [-0.1, -0.5, 0])
converyor = p.loadURDF("converyor/converyor.urdf", [0.40000, -0.30191, 0],useFixedBase=True)
camera = p.loadURDF("camera/camera.urdf", [0.883017, -0.650635, 0])

legos = list()

def update():
    tick = time.time()
    u = 1./240
        
    while True:
        t = time.time()
        dt = (t - tick)
        c =  dt / u
        tick = t - (c - int(c)) * u
        
        for i in range(int(c)):
            p.stepSimulation()


Thread(target=update).start()

def update2():
    while True:
        t = time.time()
        pos = np.array([random.uniform(0.3,0.4),-0.9,0.50])
        direction = np.array([0,0.1,0])
        lego = p.loadURDF("lego/lego.urdf",pos, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
        mass,_,_,_,_,_,_,_,_,_,_,_ = p.getDynamicsInfo(lego,-1)
        p.applyExternalForce(lego,-1,direction * mass * 240,pos,p.WORLD_FRAME)
        legos.append([t,pos,direction,lego])
        time.sleep(5)

Thread(target=update2).start()

pandaNumDofs = 7
#lower limits for null space (todo: set them to proper range)
ll = [-7]*pandaNumDofs
#upper limits for null space (todo: set them to proper range)
ul = [7]*pandaNumDofs
#joint ranges for null space (todo: set them to proper range)
jr = [7]*pandaNumDofs
#restposes for null space
rp=[0, 0, 0, 0, 0, 0.785398, 0, 0.04, 0.04]
index = 0

for j in range(p.getNumJoints(panda)):
    jointType = p.getJointInfo(panda, j)[2]
    if (jointType == p.JOINT_PRISMATIC):
        p.resetJointState(panda, j, rp[index]) 
        index=index+1
    if (jointType == p.JOINT_REVOLUTE):
        p.resetJointState(panda, j, rp[index]) 
        index=index+1

command = 'idle'

while True:
    tick = time.time()
        
    if command == 'idle':
        pos,orn,_,_,_,_ = p.getLinkState(panda,11)
        command = 'move'
        pass
    elif command == 'move':
        if not legos: continue
        t_lego,pos_lego,direction_lego,lego = legos[0]

        pos_target = pos_lego + 7 * direction_lego + [0,0,0.1]
        pos_origin = pos_prev

        direction = np.array(pos_target) - pos_origin
        direction_norm = direction / np.linalg.norm(direction)
        distance = np.linalg.norm(direction)
        distance = min(distance,1)
        speed_factor = (1-distance) * 0.2 + distance * 1.0

        if distance <= 0.01:
            pos = pos_target
            command = 'pick'
        else:
            pos = pos_origin + dt * (direction_norm * speed_factor)

        orn = p.getQuaternionFromEuler([math.pi,0,0])
        pass
    elif command == 'pick':
        t_lego,pos_lego,direction_lego,lego = legos[0]
        t = tick - t_lego

        pos_target,_ = p.getBasePositionAndOrientation(lego)
        pos_origin = pos_prev

        direction = pos_target - pos_origin
        direction_norm = direction / np.linalg.norm(direction)
        distance = np.linalg.norm(direction)
        t = min(distance,0.1) / 0.1
        speed_factor = (1 - t) * 0.13 + t * 0.2

        if distance <= 0.11:
            pos = pos_origin + dt * (direction_norm * speed_factor)
        
        if distance <= 0.001:
            for i in [9,10]: p.setJointMotorControl2(panda, i, p.POSITION_CONTROL, 0,force=10)
            command = 'place'
        
        orn = p.getQuaternionFromEuler([math.pi,0,0])
        pass
    elif command == 'place':
        pos_target = [0.002331, -0.50604, 0.599481]
        pos_origin = pos_prev

        direction = pos_target - pos_origin
        direction_norm = direction / np.linalg.norm(direction)
        distance = np.linalg.norm(direction)
        distance = min(distance,1)
        speed_factor = (1-distance) * 0.2 + distance * 1.0

        if distance <= 0.01:
            for i in [9,10]: p.setJointMotorControl2(panda, i, p.POSITION_CONTROL, 0.04,force=10)
            legos.pop(0)
            command = 'move'
        else:
            pos = pos_origin + dt * (direction_norm * speed_factor)

        orn = p.getQuaternionFromEuler([math.pi,0,0])
        pass

    if 'pos_prev' in locals(): 
        p.addUserDebugLine(pos_prev,pos,[1,0,0],lifeTime=1)

    poses = p.calculateInverseKinematics(panda,11, pos, orn,ll, ul, jr, rp, maxNumIterations=200)
    for i in range(7): p.setJointMotorControl2(panda, i, p.POSITION_CONTROL, poses[i])
    pos_prev = pos
    dt = time.time() - tick

