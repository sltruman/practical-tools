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

while True:
    scene.update()
