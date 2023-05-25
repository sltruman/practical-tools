from time import time,sleep
from digitaltwin import Scene,Workflow,Editor
import os

from threading import Thread
import numpy as np
import cv2
import digitaltwin_data

data_dir = digitaltwin_data.get_data_path()

scene = Scene(1024,768,data_dir)
editor = Editor(scene)
workflow = Workflow(scene)

scene.load(os.path.join(data_dir,'scenes/标定测试.json'))

def updating():
    import time
    while True:
        scene.update_for_tick(1/180.)
        time.sleep(1/180.)

t = Thread(target=updating)
t.start()

robot = scene.active_objs_by_name['robot']
camera = scene.active_objs_by_name['camera']
print(robot.properties())
print(camera.properties())

route_joint_positions = [
    [0.0,0,0,0,0,0.0],
    [0.1,0,0,0,0,0.1],
    [0.2,0,0,0,0,0.2],
    [0.3,0,0,0,0,0.3],
    [0.4,0,0,0,0,0.4],
    [0.5,0,0,0,0,0.5],
    [0.6,0,0,0,0,0.6]
]

for joint_pos in route_joint_positions:
    # scene.set_ground_z(scene.ground_z+0.01)
    # robot.set_joints(joint_pos)
    # camera.set_roi(joint_pos[0],0.1,0.5,0,0,0,0.1333,0.1,0.1)
    # camera.draw_roi()

    sleep(0.5)
