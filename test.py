import os
import socket as s
import json
import traceback
import sys
from time import time
# sys.path.insert(0,'/home/truman/Desktop/bullet3/build_cmake/examples/pybullet')

from digitaltwin import Scene,Editor,Workflow
scene = Scene(800,640)
editor = Editor(scene)
workflow = Workflow(scene)

scene.load('./data/scenes/无序抓取.json')
scene.start()

scene.active_objs_by_name['packer'].generate()

for _ in range(200):
    scene.update_for_tick(1/240.)

import time
while True:
    scene.update_for_tick(1/240.)
    time.sleep(1/240.)