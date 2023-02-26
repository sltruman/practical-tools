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
p.setRealTimeSimulation(1)

floor = p.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)

conveyor_forward1 = p.loadURDF("/home/truman/Desktop/digital-twin/data/end_effectors/gripper/gripper.urdf")

input()