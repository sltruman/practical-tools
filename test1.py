from time import time
from digitaltwin import Scene,Workflow

scene = Scene(1024,768)
workflow = Workflow(scene)

scene.load('./data/scenes/混合拆垛.json')
scene.active_objs_by_name['robot'].speed=1
workflow.start()

import time
while True:
    scene.update_for_tick(1/180.)
    time.sleep(1/180.)