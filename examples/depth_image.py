from time import time
import os
from digitaltwin import Scene,Workflow
import digitaltwin_data

scene = Scene(1024,768)
workflow = Workflow(scene)

data_dir = digitaltwin_data.get_data_path()
scene.load(os.path.join(data_dir,'scenes/深度图.json'))
workflow.start()

import time
while True:
    scene.update_for_tick(1/180.)
    time.sleep(1/180.)