import time
from digitaltwin import Scene,Editor, Workflow

scene = Scene(1024, 768)
scene.load('./data/scenes/空.json')
editor = Editor(scene)
workflow = Workflow(scene)
workflow.start()


while True:
    scene.update_for_tick(1/180.)
    time.sleep(1/180.)
