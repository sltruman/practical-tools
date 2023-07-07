import os
from practistyle import Scene,Workflow
import data
from threading import Thread

scene = Scene(1024,768)
workflow = Workflow(scene)

data_dir = data.get_data_path()
scene.load(os.path.join(data_dir,'scenes/标定测试.json'))

def updating():
    import time
    while True:
        scene.update_for_tick(1/180.)
        time.sleep(1/180.)

workflow.start()

t = Thread(target=updating)
t.start()
