from time import time
from digitaltwin import Scene,Workflow

scene = Scene(1024,768)
workflow = Workflow(scene)

scene.load('./data/scenes/算法插件.json')
workflow.start()

import time
while True:
    scene.update_for_tick(1/180.)
    time.sleep(1/180.)