import sys,os
sys.path.append('.')

import time
from practistyle import Scene,Editor,Renderer
import practistyle.data as data

scene = Scene(1024, 768)

data_dir = data.path()
scene.load(os.path.join(data_dir,'scenes/空.json'))
editor = Editor(scene)

obj = editor.add('Robot','robots/ur5/ur5.urdf',[0,0,0],[0,0,0])
print(obj)

renderer = Renderer(scene)

while True:
    scene.update_for_tick(1/180.)
    renderer.sync_status()
    time.sleep(1/180.)
    

