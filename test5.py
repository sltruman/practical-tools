from time import time
from digitaltwin import Scene,Workflow
from threading import Thread

scene = Scene(1024,768)
workflow = Workflow(scene)

scene.load('./scenes/标定测试.json')

def updating():
    import time
    while True:
        scene.update_for_tick(1/180.)
        time.sleep(1/180.)

t = Thread(target=updating)
t.start()
